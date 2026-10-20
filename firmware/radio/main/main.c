///=============================================================================
///
/// @copyright 2024, 2025, 2026 Todd D. Wade
///
/// @file radio_side_main.c (NimBLE version for ESP32-C6)
/// @brief Radio-side BLE server for wireless CW paddle
/// @details This ESP32-C6 acts as a BLE GATT server using NimBLE stack.
///          It receives paddle transition events from the paddle-side
///          client over BLE, and keys the radio transmitter with precise timing.
///
///=============================================================================
// clang-format off

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

// NimBLE includes
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "esp_task_wdt.h"
#include "soc/rtc.h"


//#define LOG_PADDLES                    (1)
#define LOW_POWER_MODE                 (0)  // Enable for production power savings - DISABLED FOR DEBUGGING
#define DIRECT_KEYING                  (1)  // Key GPIO directly in BLE callback for lowest latency
#define KEY_TASK_PRIORITY              (7)  // High priority for immediate keying response (unused if DIRECT_KEYING)
#define INTERNAL_ANTENNA_GPIO          (GPIO_NUM_14)

//
// Radio keying GPIOs
//
#define RADIO_KEY_GPIO                 GPIO_NUM_1   // Keys for both DIT and DAH (direct radio keying)
#define DIT_GPIO                       GPIO_NUM_0   // DIT paddle output only
#define DAH_GPIO                       GPIO_NUM_2   // DAH paddle output only

//
// This is the signal level for GPIO to use to ACTIVATE the solid-state relay.
//
#define RELAY_ACTIVE                   (0)

//
// IO control macros
//
#define SET_IO(io, state)               do {gpio_hold_dis(io);  gpio_set_level(io, state); gpio_hold_en(io); } while (0)

//
// Optional LED for visual feedback (shared with RADIO_KEY_GPIO)
//
#define USE_OPTIONAL_LED               (1)
#define LED_GPIO                       RADIO_KEY_GPIO  // LED connected to overall keying output

#ifdef USE_OPTIONAL_LED
    #define LED_ON()                   SET_IO(LED_GPIO, RELAY_ACTIVE)
    #define LED_OFF()                  SET_IO(LED_GPIO, RELAY_ACTIVE ^ 1)
    #define LED_PULSE()                do { LED_ON(); vTaskDelay(pdMS_TO_TICKS(50)); LED_OFF(); } while(0)
#else
    #define LED_ON()
    #define LED_OFF()
    #define LED_PULSE()
#endif

//
// Logging macros - in LOW_POWER_MODE, verbose logs are completely disabled
//
#ifdef LOW_POWER_MODE
    #define LOG_VERBOSE(tag, format, ...)  do {} while(0)  // No logging in low power mode
#else
    #define LOG_VERBOSE(tag, format, ...)  ESP_LOGI(tag, format, ##__VA_ARGS__)
#endif

//
// BLE Power and Connection Parameters
// Optimized for power savings while maintaining acceptable CW keying latency
//
#define BLE_TX_POWER_LEVEL             ESP_PWR_LVL_N12     // -12 dBm - low power, short range

#ifdef LOW_POWER_MODE
    #define BLE_CONN_ITVL_MIN          12                  // 15ms (12 * 1.25ms) - matches paddle-side normal interval
    #define BLE_CONN_ITVL_MAX          12                  // 15ms - same as min for consistency
    #define BLE_CONN_SLAVE_LATENCY     0                   // Don't skip events - better CW response
    #define STATS_INTERVAL_MS          120000              // 120 seconds - reduce CPU wake events
#else
    #define BLE_CONN_ITVL_MIN          12                  // 15ms (12 * 1.25ms) - matches paddle-side normal interval
    #define BLE_CONN_ITVL_MAX          12                  // 15ms - same as min for consistency
    #define BLE_CONN_SLAVE_LATENCY     0                   // Don't skip any events
    #define STATS_INTERVAL_MS          30000               // 30 seconds
#endif

#define BLE_CONN_SUPERVISION_TIMEOUT   200                 // 2 seconds (200 * 10ms) - faster reconnect detection
#define BLE_ADV_ITVL_MIN               0x0320              // 500ms (800 * 0.625ms) - saves power
#define BLE_ADV_ITVL_MAX               0x0640              // 1000ms (1600 * 0.625ms)

//
// BLE Server configuration (must match paddle-side)
//
#define RADIO_SERVICE_UUID16    0x1234
#define RADIO_CHAR_UUID16       0x5678
#define RADIO_DEVICE_NAME       "CW_Radio"

// Used for logging
static const char *TAG = "CW_RADIO";

//
// BLE Packet structure (must match paddle-side)
//
typedef struct __attribute__((packed))
{
    uint8_t paddle_type;     // 0=dit, 1=dah
    uint8_t event_type;      // 0=press, 1=release
    uint8_t mode;            // Current paddle mode
    uint16_t duration_ms;    // Press duration (0-65535ms), only meaningful for RELEASE
    uint64_t timestamp;      // Relative timing for precise reconstruction
} ble_morse_packet_t;

//
// Paddle types
//
typedef enum
{
    PADDLE_DIT = 0,
    PADDLE_DAH = 1
} paddle_type_t;

//
// Event types
//
typedef enum
{
    EVENT_PRESS = 0,
    EVENT_RELEASE = 1
} event_type_t;

//
// LED flash state machine
//
typedef enum
{
    LED_IDLE = 0,
    LED_FLASH_CONNECT,     // 3 quick flashes for connection
    LED_FLASH_DISCONNECT   // 2 slower flashes for disconnection
} led_flash_state_t;

static led_flash_state_t led_flash_state = LED_IDLE;
static uint8_t led_flash_step = 0;
static esp_timer_handle_t led_timer = NULL;

//
// NimBLE connection handle
//
static uint16_t conn_handle = 0;
static bool client_connected = false;
static uint8_t own_addr_type;

//
// Timing tracking for precise reconstruction
//
static uint64_t last_packet_timestamp = 0;

//
// Statistics
//
static struct
{
    uint32_t packets_received;
    uint32_t packets_dropped;      // Track dropped packets when queue full
    uint32_t queue_high_water;     // Track maximum queue depth
    uint32_t dits_received;
    uint32_t dahs_received;
    uint32_t connections;
    uint32_t disconnections;
} stats = {0};

// Spinlock for protecting stats and LED state from concurrent access
static portMUX_TYPE stats_spinlock = portMUX_INITIALIZER_UNLOCKED;

//
// Async packet processing queue
//
#define PACKET_QUEUE_SIZE 32
static QueueHandle_t packet_queue = NULL;

// Task handles for potential graceful shutdown
static TaskHandle_t keying_task_handle = NULL;
static TaskHandle_t stats_task_handle = NULL;

//
// Keying timing state
//
static uint64_t last_key_end_time = 0;

// =============================================
// INTERNAL PROTOTYPES
// =============================================

static void startAdvertising(void);

// =============================================
// LED FLASH STATE MACHINE
// =============================================

// Flash pattern timing (microseconds)
#define LED_FLASH_FAST_US      80000   // 80ms for quick flashes
#define LED_FLASH_SLOW_US      350000  // 200ms for slow flashes

///
/// @fn: ledTimerCallback
/// @details One-shot timer callback that advances the LED flash state machine.
///          Schedules next step only if pattern is incomplete, preserving power.
///
/// @param[in] arg - timer callback argument (unused)
///
/// @returns none
///
static void ledTimerCallback(void *arg)
{
    bool led_on = (led_flash_step % 2) == 0;  // Even steps = ON, odd = OFF
    uint32_t next_delay_us = 0;
    bool pattern_complete = false;

    switch (led_flash_state)
    {
        case LED_FLASH_CONNECT:
            // Pattern: ON-off-ON-off-ON-off (6 steps, 3 flashes)
            if (led_flash_step >= 6)
            {
                pattern_complete = true;
            }
            else
            {
                if (led_on) LED_ON(); else LED_OFF();
                next_delay_us = LED_FLASH_FAST_US;
            }
            break;

        case LED_FLASH_DISCONNECT:
            // Pattern: ON-off-ON-off (4 steps, 2 flashes)
            if (led_flash_step >= 4)
            {
                pattern_complete = true;
            }
            else
            {
                if (led_on) LED_ON(); else LED_OFF();
                next_delay_us = LED_FLASH_SLOW_US;
            }
            break;

        case LED_IDLE:
        default:
            pattern_complete = true;
        break;
    }

    if (pattern_complete)
    {
        led_flash_state = LED_IDLE;
        led_flash_step = 0;
        LED_OFF();
        // Timer stops here - no more wakeups until next event
    }
    else
    {
        led_flash_step++;
        esp_timer_start_once(led_timer, next_delay_us);
    }
}

///
/// @fn: startLedPattern
/// @details Initiates an LED flash pattern. Uses one-shot timer to minimize
///          power consumption - timer only runs during active pattern.
///
/// @param[in] pattern - the flash pattern to start
///
/// @returns none
///
static void startLedPattern(led_flash_state_t pattern)
{
#ifdef USE_OPTIONAL_LED
    // Protect LED state transitions from concurrent access
    portENTER_CRITICAL(&stats_spinlock);

    // Stop any in-progress pattern
    if (led_timer != NULL)
    {
        esp_timer_stop(led_timer);
    }

    led_flash_state = pattern;
    led_flash_step = 0;

    portEXIT_CRITICAL(&stats_spinlock);

    // Start pattern with LED on, schedule first toggle
    LED_ON();
    led_flash_step++;

    uint32_t first_delay = (pattern == LED_FLASH_CONNECT) ? LED_FLASH_FAST_US : LED_FLASH_SLOW_US;
    esp_timer_start_once(led_timer, first_delay);
#endif
}

#ifdef LOW_POWER_MODE

///
/// @fn: disableUARTConsole
/// @details Disables UART console output to save 5-10 mA power consumption.
///          Call after initialization is complete.
///
/// @param[in] none
///
/// @returns none
///
static void disableUARTConsole(void)
{
    // Flush any pending output
    uart_wait_tx_done(CONFIG_ESP_CONSOLE_UART_NUM, pdMS_TO_TICKS(100));

    // Disable UART TX to save power (console becomes silent)
    uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);

    ESP_LOGI(TAG, "UART console disabled for power savings (last message)");
}
#endif

// =============================================
// GPIO CONFIGURATION
// =============================================

///
/// @fn: initGPIO
/// @details Configures GPIO pins for radio keying outputs (RADIO_KEY, DIT, DAH),
///          internal antenna selection, and optional LED indicator
///
/// @param[in] none
///
/// @returns none
///
static void initGPIO(void)
{
    // Configure radio keying outputs (all three)
    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL << RADIO_KEY_GPIO) | (1ULL << DIT_GPIO) | (1ULL << DAH_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    //
    // Set initial state of paddle IO so that we don't key the
    // rig on power-up, then hold the levels.
    //
    // GPIO0-GPIO2 are LP GPIO pins on ESP32-C6. When light sleep is enabled,
    // the LP GPIO controller takes over these pads during sleep and resets them
    // to input/low unless gpio_hold_en() is called. Hold must be released before
    // each gpio_set_level() and re-enabled afterward.
    //
    SET_IO(RADIO_KEY_GPIO, RELAY_ACTIVE ^ 1);
    SET_IO(DIT_GPIO, RELAY_ACTIVE ^ 1);
    SET_IO(DAH_GPIO, RELAY_ACTIVE ^ 1);

    //
    // Configure internal antenna for XIAO ESP32C6 - reuse io_conf
    // GPIO 3 = LOW enables RF switch, GPIO 14 = LOW selects internal PCB antenna
    //
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << INTERNAL_ANTENNA_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    SET_IO(GPIO_NUM_3, 0);

    vTaskDelay(pdMS_TO_TICKS(10));   // Brief delay for RF switch

    gpio_set_level(INTERNAL_ANTENNA_GPIO, 0);  // LOW = internal PCB antenna
    LOG_VERBOSE(TAG, "Internal antenna configured (GPIO3=LOW, GPIO14=LOW)");

    // Create one-shot timer for LED flash patterns (power-efficient)
    // Note: LED_GPIO shares RADIO_KEY_GPIO, already configured above
#ifdef USE_OPTIONAL_LED
    const esp_timer_create_args_t led_timer_args = {
        .callback = ledTimerCallback,
        .name = "led_flash"
    };
    esp_timer_create(&led_timer_args, &led_timer);
#endif
    LOG_VERBOSE(TAG, "GPIO configured: KEY=%d, DIT=%d, DAH=%d, LED=%d",
             RADIO_KEY_GPIO, DIT_GPIO, DAH_GPIO, LED_GPIO);
    ESP_LOGI(TAG, "initGPIO() complete");
}

// =============================================
// PADDLE PACKET PROCESSING
// =============================================

///
/// @fn: processPaddlePacket
/// @details Processes paddle packet - either keys GPIO directly in BLE callback
///          (DIRECT_KEYING) or queues for async processing.
///
/// @param[in] data - pointer to packet data
/// @param[in] len - length of packet data in bytes
///
/// @returns none
///
static void processPaddlePacket(const uint8_t *data, uint16_t len)
{
    if (len != sizeof(ble_morse_packet_t))
    {
        return;  // Drop silently - can't afford logging here
    }

    const ble_morse_packet_t *packet = (const ble_morse_packet_t *)data;
    portENTER_CRITICAL(&stats_spinlock);
    stats.packets_received++;
    portEXIT_CRITICAL(&stats_spinlock);

#if (DIRECT_KEYING==1)
    //
    // Key GPIO directly in BLE callback for lowest latency.
    // gpio_hold_dis/en brackets required: GPIO0-GPIO2 are LP GPIO on ESP32-C6
    // and gpio_set_level() is ignored while hold is active.
    //
    if (packet->event_type == EVENT_PRESS)
    {
        SET_IO(RADIO_KEY_GPIO, RELAY_ACTIVE);
        if (packet->paddle_type == PADDLE_DIT)
        {
            SET_IO(DIT_GPIO, RELAY_ACTIVE);

            portENTER_CRITICAL(&stats_spinlock);
            stats.dits_received++;
            portEXIT_CRITICAL(&stats_spinlock);
        }
        else
        {
            SET_IO(DAH_GPIO, RELAY_ACTIVE);

            portENTER_CRITICAL(&stats_spinlock);
            stats.dahs_received++;
            portEXIT_CRITICAL(&stats_spinlock);
        }
    }
    else // EVENT_RELEASE
    {
        SET_IO(RADIO_KEY_GPIO, RELAY_ACTIVE ^ 1);
        SET_IO(DIT_GPIO, RELAY_ACTIVE ^ 1);
        SET_IO(DAH_GPIO, RELAY_ACTIVE ^ 1);
    }
#else
    // Queue packet for async processing
    if (xQueueSend(packet_queue, packet, 0) != pdTRUE)
    {
        portENTER_CRITICAL(&stats_spinlock);
        stats.packets_dropped++;
        portEXIT_CRITICAL(&stats_spinlock);
    }
    else
    {
        // Track queue high water mark (fast operation)
        UBaseType_t queue_depth = uxQueueMessagesWaiting(packet_queue);
        portENTER_CRITICAL(&stats_spinlock);
        if (queue_depth > stats.queue_high_water)
        {
            stats.queue_high_water = queue_depth;
        }
        portEXIT_CRITICAL(&stats_spinlock);
    }
#endif
}

// =============================================
// NimBLE GATT SERVER
// =============================================

// Characteristic value handle (filled in during registration)
static uint16_t char_val_handle;

///
/// @fn: gatt_char_access
/// @details GATT characteristic access callback for write and read operations.
///          Handles incoming paddle packets from BLE client.
///
/// @param[in] conn_handle_param - BLE connection handle
/// @param[in] attr_handle - attribute handle
/// @param[in] ctxt - GATT access context containing operation and data
/// @param[in] arg - user argument (unused)
///
/// @returns int - 0 on success, BLE error code on failure
///
static int gatt_char_access(uint16_t conn_handle_param, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int             ret = 0;

    switch (ctxt->op)
    {
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // CRITICAL: No logging here - must be ultra-fast!
            processPaddlePacket(ctxt->om->om_data, ctxt->om->om_len);
            ret = 0;
        break;

        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Could return statistics or status here if needed
            ret = 0;
        break;

        default:
            ret = BLE_ATT_ERR_UNLIKELY;
        break;
    }

    return ret;
}

// GATT service definition
static const struct ble_gatt_svc_def gatt_svcs[] =
{
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(RADIO_SERVICE_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = BLE_UUID16_DECLARE(RADIO_CHAR_UUID16),
                .access_cb = gatt_char_access,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_READ,
                .val_handle = &char_val_handle,
            },
            {
                0, // No more characteristics
            }
        },
    },
    {
        0, // No more services
    },
};

// =============================================
// NimBLE GAP EVENT HANDLING
// =============================================

///
/// @fn: gap_event_handler
/// @details Handles NimBLE GAP events including connection, disconnection,
///          advertising completion, and connection parameter updates
///
/// @param[in] event - pointer to GAP event structure
/// @param[in] arg - user argument (unused)
///
/// @returns int - 0 on success
///
static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    LOG_VERBOSE(TAG, "GAP event: type=%d", event->type);

    switch (event->type)
    {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE_GAP_EVENT_CONNECT: status=%d", event->connect.status);
            ESP_LOGI(TAG, "BLE connection %s; status=%d",
                     event->connect.status == 0 ? "established" : "failed",
                     event->connect.status);

            if (event->connect.status == 0)
            {
                ESP_LOGI(TAG, ">>> Advertising STOPPED (client connected)");
                conn_handle = event->connect.conn_handle;
                client_connected = true;
                portENTER_CRITICAL(&stats_spinlock);
                stats.connections++;
                portEXIT_CRITICAL(&stats_spinlock);
                startLedPattern(LED_FLASH_CONNECT);  // 3 quick flashes
                // Connection interval is managed by the paddle side (central),
                // which adapts it based on detected CW speed. Do not counter-request here.
            }
        break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, ">>> BLE disconnected; reason=%d (restarting advertising)", event->disconnect.reason);
            client_connected = false;
            portENTER_CRITICAL(&stats_spinlock);
            stats.disconnections++;
            portEXIT_CRITICAL(&stats_spinlock);
            startLedPattern(LED_FLASH_DISCONNECT);  // 2 slower flashes

            // Restart advertising
            startAdvertising();
        break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            LOG_VERBOSE(TAG, "Advertising complete; reason=%d", event->adv_complete.reason);
            // Only restart advertising if not already connected
            if (!client_connected) {
                startAdvertising();
            }
        break;

        case BLE_GAP_EVENT_CONN_UPDATE:
        {
            struct ble_gap_conn_desc desc;
            int rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
            if (rc == 0)
            {
                ESP_LOGI(TAG, "Conn params updated: interval=%d (%.1fms), latency=%d, timeout=%d",
                         desc.conn_itvl, desc.conn_itvl * 1.25,
                         desc.conn_latency, desc.supervision_timeout);
            }
            else
            {
                ESP_LOGW(TAG, "Conn update status=%d, but couldn't get params (rc=%d)",
                         event->conn_update.status, rc);
            }
        }
        break;

        case BLE_GAP_EVENT_MTU:
            LOG_VERBOSE(TAG, "MTU update event; conn_handle=%d mtu=%d",
                     event->mtu.conn_handle, event->mtu.value);
        break;
    }

    return 0;
}

// =============================================
// BLE ADVERTISING
// =============================================

///
/// @fn: startAdvertising
/// @details Configures and starts BLE advertising with device name and service UUID.
///          Makes radio side discoverable to paddle side client.
///
/// @param[in] none
///
/// @returns none
///
static void startAdvertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    LOG_VERBOSE(TAG, ">>> startAdvertising() called");

    memset(&fields, 0, sizeof(fields));

    // Set flags: General discoverable + BR/EDR not supported
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Set device name in advertising data
    fields.name = (uint8_t *)RADIO_DEVICE_NAME;
    fields.name_len = strlen(RADIO_DEVICE_NAME);
    fields.name_is_complete = 1;

    LOG_VERBOSE(TAG, "Setting advertising name to: %s (len=%d)", RADIO_DEVICE_NAME, fields.name_len);

    // Set service UUID
    fields.uuids16 = (ble_uuid16_t[])
    {
        BLE_UUID16_INIT(RADIO_SERVICE_UUID16)
    };

    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set advertising data; rc=%d", rc);
        return;
    }

    LOG_VERBOSE(TAG, "Advertising fields set successfully");

    // Begin advertising with proper parameters
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_ADV_ITVL_MIN;  // 500ms - power savings
    adv_params.itvl_max = BLE_ADV_ITVL_MAX;  // 1000ms

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, ">>> Advertising ACTIVE (interval: %d-%dms)",
             (BLE_ADV_ITVL_MIN * 625) / 1000, (BLE_ADV_ITVL_MAX * 625) / 1000);
}

// =============================================
// NimBLE STACK INITIALIZATION
// =============================================

///
/// @fn: on_sync
/// @details Called when NimBLE host is synchronized and ready.
///          Configures address, prints device info, and starts advertising.
///
/// @param[in] none
///
/// @returns none
///
static void on_sync(void)
{
    int rc;

    ESP_LOGI(TAG, "NimBLE host synchronized");

    // Ensure we have a valid address
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to ensure address; rc=%d", rc);
        return;
    }

    // Figure out address to use while advertising (no privacy for now)
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type; rc=%d", rc);
        return;
    }

    // Print our BLE address
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    ESP_LOGI(TAG, "Device Address: %02x:%02x:%02x:%02x:%02x:%02x (type=%d)",
             addr_val[5], addr_val[4], addr_val[3],
             addr_val[2], addr_val[1], addr_val[0],
             own_addr_type);

    ESP_LOGI(TAG, "Starting advertising as: %s", RADIO_DEVICE_NAME);

    // Begin advertising
    startAdvertising();
}

///
/// @fn: on_reset
/// @details Called when NimBLE host resets due to error or controller reset
///
/// @param[in] reason - reset reason code
///
/// @returns none
///
static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset; reason=%d", reason);
}

///
/// @fn: nimble_host_task
/// @details FreeRTOS task that runs the NimBLE host stack
///
/// @param[in] param - task parameter (unused)
///
/// @returns none
///
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

///
/// @fn: initBLE
/// @details Initializes NimBLE stack as GATT server, configures TX power,
///          registers services and characteristics, and starts host task
///
/// @param[in] none
///
/// @returns none
///
static void initBLE(void)
{
    int rc;

    // Configure power management for reduced power consumption
    esp_pm_config_t pm_config =
    {
        .max_freq_mhz = 80,  // Maximum CPU frequency
        .min_freq_mhz = 40,   // Minimum CPU frequency during light sleep
        .light_sleep_enable = true   // Enabled: may add 1-3ms wake latency, test for acceptability
    };
    ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    ESP_LOGI(TAG, "Power management configured: CPU 80/40 MHz, light sleep enabled");

    // Initialize NVS (required for BLE)
    esp_err_t ret = nvs_flash_init();
    if ( (ret == ESP_ERR_NVS_NO_FREE_PAGES) || (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) )
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    // Initialize NimBLE
    ESP_ERROR_CHECK(nimble_port_init());

    // Set TX power for balance between range and power consumption
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BLE_TX_POWER_LEVEL));
    ESP_ERROR_CHECK(esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_POWER_LEVEL));
    ESP_LOGI(TAG, "BLE TX power configured for power savings (0 dBm)");

    // Initialize the NimBLE host configuration
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    // Initialize standard GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Initialize GATT services
    rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to count GATT services; rc=%d", rc);
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to add GATT services; rc=%d", rc);
    }

    // Set device name in GAP service
    rc = ble_svc_gap_device_name_set(RADIO_DEVICE_NAME);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to set device name; rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "GAP device name set to: %s", RADIO_DEVICE_NAME);
    }

    // Start the NimBLE host task
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "NimBLE initialized successfully");
}

// =============================================
// STATISTICS TASK
// =============================================

///
/// @fn: statsTask
/// @details FreeRTOS task that periodically logs connection and packet statistics
///
/// @param[in] param - task parameter (unused)
///
/// @returns none (does not return - runs forever)
///
static void statsTask(void *param)
{
    static uint32_t stats_cycles = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(STATS_INTERVAL_MS));

        // Get current CPU frequency
        rtc_cpu_freq_config_t freq_config;
        rtc_clk_cpu_freq_get_config(&freq_config);

        // Take a snapshot of stats under lock
        uint32_t local_connections, local_disconnections;
        uint32_t local_packets_received, local_packets_dropped;
        uint32_t local_queue_high_water, local_dits, local_dahs;

        portENTER_CRITICAL(&stats_spinlock);
        local_connections = stats.connections;
        local_disconnections = stats.disconnections;
        local_packets_received = stats.packets_received;
        local_packets_dropped = stats.packets_dropped;
        local_queue_high_water = stats.queue_high_water;
        local_dits = stats.dits_received;
        local_dahs = stats.dahs_received;
        portEXIT_CRITICAL(&stats_spinlock);

        ESP_LOGI(TAG, "=== Statistics ===");
        ESP_LOGI(TAG, "CPU Frequency: %" PRIu32 " MHz", freq_config.freq_mhz);
        ESP_LOGI(TAG, "Connected: %s", client_connected ? "YES" : "NO");
        ESP_LOGI(TAG, "Connections: %" PRIu32, local_connections);
        ESP_LOGI(TAG, "Disconnections: %" PRIu32, local_disconnections);
        ESP_LOGI(TAG, "Packets: %" PRIu32 " (dropped: %" PRIu32 ")",
                 local_packets_received, local_packets_dropped);
        ESP_LOGI(TAG, "Queue High Water: %" PRIu32 " / %d", local_queue_high_water, PACKET_QUEUE_SIZE);
        ESP_LOGI(TAG, "DITs: %" PRIu32 ", DAHs: %" PRIu32, local_dits, local_dahs);

        // Reset packet counters periodically to prevent overflow (keep connections/disconnections)
        // 720 cycles at 120s = ~24 hours, or 720 cycles at 30s = ~6 hours
        if (++stats_cycles >= 720)
        {
            ESP_LOGI(TAG, "Resetting packet counters (periodic cycle)");
            portENTER_CRITICAL(&stats_spinlock);
            stats.packets_received = 0;
            stats.packets_dropped = 0;
            stats.queue_high_water = 0;
            stats.dits_received = 0;
            stats.dahs_received = 0;
            portEXIT_CRITICAL(&stats_spinlock);
            stats_cycles = 0;
        }
    }
}

// =============================================
// RADIO KEYING TASK (ASYNC FROM BLE)
// =============================================

#if (DIRECT_KEYING==0)

///
/// @fn: radioKeyingTask
/// @details FreeRTOS task that processes queued paddle packets and controls
///          radio keying outputs. Runs asynchronously from BLE to maintain
///          fast BLE response while allowing precise timing control.
///
/// @param[in] pvParameters - task parameter (unused)
///
/// @returns none (does not return - runs forever)
///
static void radioKeyingTask(void *pvParameters)
{
    ble_morse_packet_t packet;

    ESP_LOGI(TAG, "Radio keying task started (press/release mode)");

    while (1)
    {
        // Wait for packet from queue (this task blocks, but BLE doesn't!)
        if (xQueueReceive(packet_queue, &packet, portMAX_DELAY) == pdTRUE)
        {
        #ifndef LOW_POWER_MODE
            const char* paddle_str = (packet.paddle_type == PADDLE_DIT) ? "DIT" : "DAH";
            const char* event_str = (packet.event_type == EVENT_PRESS) ? "PRESS" : "RELEASE";
            LOG_VERBOSE(TAG, "Processing: %s %s (ts=%" PRIu64 ")", event_str, paddle_str, packet.timestamp);
        #endif

            if (packet.event_type == EVENT_PRESS)
            {
                // PRESS event - key transmitter ON immediately
                SET_IO(RADIO_KEY_GPIO, RELAY_ACTIVE);

                if (packet.paddle_type == PADDLE_DIT)
                {
                    SET_IO(DIT_GPIO, RELAY_ACTIVE);
                    portENTER_CRITICAL(&stats_spinlock);
                    stats.dits_received++;
                    portEXIT_CRITICAL(&stats_spinlock);
                }
                else
                {
                    SET_IO(DAH_GPIO, RELAY_ACTIVE);
                    portENTER_CRITICAL(&stats_spinlock);
                    stats.dahs_received++;
                    portEXIT_CRITICAL(&stats_spinlock);
                }

            #ifndef LOW_POWER_MODE
                LOG_VERBOSE(TAG, "Keyed ON: %s", paddle_str);
            #endif
            }
            else // EVENT_RELEASE
            {
                // RELEASE event - key transmitter OFF immediately
                SET_IO(RADIO_KEY_GPIO, RELAY_ACTIVE ^ 1);
                SET_IO(DIT_GPIO, RELAY_ACTIVE ^ 1);
                SET_IO(DAH_GPIO, RELAY_ACTIVE ^ 1);

            #ifndef LOW_POWER_MODE
                LOG_VERBOSE(TAG, "Keyed OFF: %s (duration was %d ms)", paddle_str, packet.duration_ms);
            #endif
            }

            last_packet_timestamp = packet.timestamp;
            last_key_end_time = esp_timer_get_time() / 1000;
        }
    }
}
#endif // DIRECT_KEYING==0

// =============================================
// MAIN
// =============================================

///
/// @fn: app_main
/// @details Main entry point for radio side application. Initializes GPIO,
///          creates packet queue, starts keying task, initializes BLE,
///          and starts statistics task.
///
/// @param[in] none
///
/// @returns none
///
void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Wireless CW Paddle - Radio Side");
    ESP_LOGI(TAG, "  BLE GATT Server (NimBLE/ESP32-C6)");
    ESP_LOGI(TAG, "========================================");

    // Initialize hardware
    ESP_LOGI(TAG, "Initializing GPIO...");
    initGPIO();
    ESP_LOGI(TAG, "GPIO initialization complete");

#if (DIRECT_KEYING==1)
    ESP_LOGI(TAG, "Direct keying mode - GPIO keyed in BLE callback");
#else
    // Create async packet queue
    ESP_LOGI(TAG, "Creating packet queue...");
    packet_queue = xQueueCreate(PACKET_QUEUE_SIZE, sizeof(ble_morse_packet_t));
    if (packet_queue == NULL)
    {
        ESP_LOGE(TAG, "Failed to create packet queue");
        return;
    }
    ESP_LOGI(TAG, "Packet queue created");

    // Start async radio keying task
    ESP_LOGI(TAG, "Starting radio keying task...");
    xTaskCreate(radioKeyingTask, "radio_key", 4096, NULL, KEY_TASK_PRIORITY, &keying_task_handle);
    ESP_LOGI(TAG, "Radio keying task started");
#endif

    // Initialize BLE
    ESP_LOGI(TAG, "Initializing BLE...");
    initBLE();
    ESP_LOGI(TAG, "BLE initialization complete");

    // Start statistics task (2560 bytes for logging + local structs)
#ifndef LOW_POWER_MODE
    ESP_LOGI(TAG, "Starting statistics task...");
    xTaskCreate(statsTask, "stats", 2560, NULL, 5, &stats_task_handle);
    ESP_LOGI(TAG, "Statistics task started");
#else
    ESP_LOGI(TAG, "Statistics task disabled (low power mode)");
#endif
    ESP_LOGI(TAG, "Radio-side initialization complete");
    ESP_LOGI(TAG, "Waiting for paddle-side connection...");

#ifdef LOW_POWER_MODE
    // Check power management locks before disabling console
    vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for system to settle

    ESP_LOGI(TAG, "=== Power Management Status ===");
    esp_pm_dump_locks(stdout);

    // Disable UART console for power savings (5-10 mA)
    vTaskDelay(pdMS_TO_TICKS(500));  // Brief delay to ensure last log prints
    disableUARTConsole();
#else

#endif
}
