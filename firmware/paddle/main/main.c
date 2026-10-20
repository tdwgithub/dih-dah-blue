///=============================================================================
///
/// @copyright 2024, 2025, 2026 Todd D. Wade
///
/// @file main.c
/// @brief High-power core code.  Implements power-management, BLE, timer
///        management, etc.
/// @details
///   This runs on the high-power core and is the focal-point of the paddle
///   side application.  It sets up the BLE connection with the radio side
///   and handles power-management.  Power saving for battery operation
///   uses the following behavior tree:
///
///   - Full power:
///      - BLE set up, connection
///      - Configure ULP, queue management
///      - Monitors queue and sends paddle notifications to radio side.
///      - Consumes about 40 mA.
///      - After some idle period, sets up a wakeup timer that is used to
///        wake up the HP core after some period of no activity.
///          - Enters light-sleep mode.
///              - in light-sleep mode, the ULP will wake up the HP core when
///                there are paddle transitions.
///              - in light-sleep mode, current consumption is ~= 200uA
///          - If in light-sleep mode and the wakeup timer expires, wakes up
///            the HP core. HP core examines wake-up cause and if its the timer,
///            enters deep-sleep mode. In that mode, the device uses the lowest
///            power.  ULP continues operating and if a paddle transition
///            occurs, wakes the HP core.
///          - in deep-sleep mode, current consumption is ~= 25uA
///
///=============================================================================
// clang-format off

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rtc_io.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_bt.h"
#include "esp_pm.h"

// NimBLE includes
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "host/ble_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_rom_uart.h"
#include "ulp_lp_core.h"

//
// BLE Tx Level Values
//
// #define BLE_POWER_LEVEL                 ESP_PWR_LVL_P3
#define BLE_POWER_LEVEL                 ESP_PWR_LVL_N12
// #define BLE_POWER_LEVEL                 ESP_PWR_LVL_N9
// #define BLE_POWER_LEVEL                 ESP_PWR_LVL_N3

// Power optimization: Disable UART logging to save 1-3mA (set to 0 for production)
#define ENABLE_UART_LOGGING                (1)

// Enable/disable automatic light-sleep mode
// NOTE: Auto BLE sleep interferes with active scanning on central/client role
#define USE_AUTO_BLE_SLEEP                 (0)

// Log paddle activity if defined
// #define LOG_PADDLES                     (1)

//
// Other constants
//
#define BLE_GAP_CONNECT_TIMEOUT_MS         (5000)
#define BLE_GAP_DISCONNECT_TIMEOUT_MS      (5000)
#define ULP_POLLING_INTERVAL               (10000)  // 10ms polling (power optimized, still very responsive)
#define QUEUE_TASK_PRIORITY                (6)
#define FAST_CW_THRESHOLD_MS               (30)     // Press duration below this → ~40+ WPM
#define FAST_CW_REVERT_MS                  (1000)   // Revert to normal interval after 1s without fast events
#define BLE_ITVL_NORMAL                    (12)     // 15ms (12 × 1.25ms) - power-optimized
#define BLE_ITVL_FAST                      (6)      // 7.5ms (6 × 1.25ms) - minimum latency for fast CW

//
// Include the auto-generated ULP header (this will be created by build system)
//
#include "ulp_main.h"

//
// Share structures between HP and ULP cores
//
#include "ulp/shared_data.h"

//
// Configuration Button IO. (Not currently doing anything)
// Define button GPIO since it's missing from shared_data.h
//
#define BUTTON_GPIO                        GPIO_NUM_9

//
// Optional LED definitions
//
#if (USE_HP_LED==1)
    #define LED_GPIO                       GPIO_NUM_2
    #define HP_LED_ON()                    gpio_set_level(LED_GPIO, 0)
    #define HP_LED_OFF()                   gpio_set_level(LED_GPIO, 1)
#else
    #define HP_LED_ON()
    #define HP_LED_OFF()
#endif

//
// Our app label
//
static const char *TAG = "WIRELESS_PADDLE";

//
// Needed to integrate the ULP program
//
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

//
// ULP Shared variables
//
extern uint32_t             ulp_morse_queue;
extern uint32_t             ulp_wakeup_count;
volatile morse_queue_t*     morse_queue = (volatile morse_queue_t*)&ulp_morse_queue;

// =============================================
// SYSTEM STATE & CONFIGURATION
// =============================================

//
// What state are we transitioning FROM
//
typedef enum
{
    SYSTEM_STATE_ACTIVE,
    SYSTEM_STATE_LIGHT_SLEEP,
    SYSTEM_STATE_DEEP_SLEEP
} system_state_t;

//
// We will be able to reverse DIT/DAH assignments
// and IAMBIC modes
//
typedef enum
{
    PADDLE_MODE_NORMAL,
    PADDLE_MODE_REVERSED,
    PADDLE_MODE_IAMBIC_A,
    PADDLE_MODE_IAMBIC_B
} paddle_mode_t;

//
// Timers that can be configured and whose
// timeout values are stored in configuration
// FLASH.
//
typedef struct
{
    uint32_t        light_sleep_timeout_ms;
    uint32_t        deep_sleep_timeout_ms;
    paddle_mode_t   paddle_mode;
    uint16_t        ble_tx_power;
} config_t;

//
// Power management state tracking
//
typedef struct
{
    system_state_t      current_state;
    uint64_t            last_activity_time;
    bool                ble_connected;
    config_t            config;
} power_mgr_t;

//
// Persistent state across deep sleep cycles
//
RTC_DATA_ATTR struct
{
    bool            ulp_initialized;
    bool            first_boot;
    uint32_t        deep_sleep_count;
    uint32_t        total_wakeups;
} rtc_state = {0};

//
// Persistent BLE connection state across light/deep sleep for fast reconnection
//
RTC_DATA_ATTR struct
{
    ble_addr_t      radio_mac_addr;
    bool            radio_mac_cached;
} rtc_ble_cache = {0};

//
// Global HP core state
//
static power_mgr_t          power_mgr = {0};
static EventGroupHandle_t   system_events;

// Event bits
#define PADDLE_ACTIVITY_BIT                BIT0
#define BUTTON_PRESSED_BIT                 BIT1
#define BLE_CONNECTED_BIT                  BIT2
#define BLE_DISCONNECTED_BIT               BIT3

// =============================================
// BLE DEFINITIONS
// =============================================

// BLE service UUID and characteristics (matches radio-side server)
#define PADDLE_SERVICE_UUID16      RADIO_SERVICE_UUID16
#define PADDLE_CHAR_UUID16         RADIO_CHAR_UUID16

//
// Paddle state transitions that get sent to the
// radio-side peer
// Optimized packet format for morse data
//
typedef struct __attribute__((packed))
{
    uint8_t     paddle_type;     // 0=dit, 1=dah
    uint8_t     event_type;      // 0=press, 1=release
    uint8_t     mode;            // Current paddle mode
    uint16_t    duration_ms;    // Press duration (0-65535ms), only meaningful for RELEASE
    uint64_t    timestamp;      // Relative timing for precise reconstruction (no overflow)
} ble_morse_packet_t;


//
// BLE Client (GATT Central) variables - NimBLE
//
static uint16_t                 conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t                 service_start_handle = 0;
static uint16_t                 service_end_handle = 0;
static uint16_t                 char_handle = 0;
static volatile bool            is_scanning = false;    // Accessed from BLE callbacks and main task
static volatile bool            is_connecting = false;  // Accessed from BLE callbacks and main task
static uint8_t                  own_addr_type;
static esp_pm_lock_handle_t     ble_pm_lock = NULL;
static TaskHandle_t             queue_task_handle = NULL;
static bool                     ble_fast_mode = false;
static uint64_t                 last_fast_event_ms = 0;

// LED flash state machine for non-blocking connection indication
static volatile bool            connection_flash_pending = false;
static uint8_t                  connection_flash_count = 0;
static uint64_t                 connection_flash_next_time = 0;

//
// Prototypes
//
static void enterDeepSleepMode(void);

// NimBLE callbacks
static int nimble_gap_event(struct ble_gap_event *event, void *arg);
static int gatt_disc_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_svc *service, void *arg);
static int gatt_disc_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                           const struct ble_gatt_chr *chr, void *arg);
static void nimble_on_sync(void);
static void nimble_on_reset(int reason);
static void nimble_host_task(void *param);
static void startScanAndConnect(void);

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                   NVS-Based Configuration Storage/Retrieval
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: loadConfigFromNVS
/// @details Pulls in the configuration data from the NVS
///          storage.
///
/// @param[in] none
///
/// @returns none
///
static void loadConfigFromNVS(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("paddle_config", NVS_READONLY, &nvs_handle);

    if (err == ESP_OK)
    {
        size_t          required_size = sizeof(config_t);

        err = nvs_get_blob(nvs_handle, "config", &power_mgr.config, &required_size);
        nvs_close(nvs_handle);
    }

    if (err != ESP_OK)
    {
        // Load defaults
        power_mgr.config.light_sleep_timeout_ms = LIGHT_SLEEP_TIMEOUT_MS;
        power_mgr.config.deep_sleep_timeout_ms = DEEP_SLEEP_TIMEOUT_MS;
        power_mgr.config.paddle_mode = PADDLE_MODE_NORMAL;
        power_mgr.config.ble_tx_power = BLE_POWER_LEVEL;   // ESP_PWR_LVL_P3;    // +3dBm

        ESP_LOGI(TAG, "Using default configuration");
    }
    else
    {
        ESP_LOGI(TAG, "Configuration loaded from NVS");
    }
}

///
/// @fn: saveConfigToNVS
/// @details Save current configuration to the NVS
///          storage.
///
/// @param[in] none
///
/// @returns none
///
static void saveConfigToNVS(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("paddle_config", NVS_READWRITE, &nvs_handle);

    if (err == ESP_OK)
    {
        err = nvs_set_blob(nvs_handle, "config", &power_mgr.config, sizeof(config_t));
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Configuration saved to NVS");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to save configuration: %s", esp_err_to_name(err));
    }
}

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                       BLE Configuration & Handling
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: nimble_gap_event
/// @details Handles NimBLE GAP events (scanning, connection)
///
/// @param[in] event - pointer to GAP event structure
/// @param[in] arg - user argument (unused)
///
/// @returns integer, always zero.
///
static int nimble_gap_event(struct ble_gap_event *event, void *arg)
{
    struct          ble_gap_conn_desc desc;
    int             rc;

    switch (event->type)
    {
        case BLE_GAP_EVENT_DISC:
            // Device discovered during scan
            {
                const struct ble_hs_adv_field *name_field = NULL;
                uint8_t *name_data = NULL;
                uint8_t name_len = 0;

                // Parse advertising data for complete local name
                int find_rc = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_COMP_NAME,
                                                    event->disc.data,
                                                    event->disc.length_data,
                                                    &name_field);

                if (find_rc == 0 && name_field)
                {
                    name_data = (uint8_t *)name_field->value;
                    name_len = name_field->length;
                }
                else
                {
                    // Try short name
                    find_rc = ble_hs_adv_find_field(BLE_HS_ADV_TYPE_INCOMP_NAME,
                                                    event->disc.data,
                                                    event->disc.length_data,
                                                    &name_field);

                    if (find_rc == 0 && name_field)
                    {
                        name_data = (uint8_t *)name_field->value;
                        name_len = name_field->length;
                    }
                }

                // Log devices with names OR our target MAC
                if (name_data)
                {
                    ESP_LOGI(TAG, "BLE device: Name=%.*s RSSI=%d",
                             name_len, name_data, event->disc.rssi);
                }

                //
                // Check if this is the radio server
                // Note: NimBLE may include null terminator, so check if name starts with our target
                //
                if (
                        (name_data) &&
                        (name_len >= strlen(RADIO_DEVICE_NAME)) &&
                        (memcmp(name_data, RADIO_DEVICE_NAME, strlen(RADIO_DEVICE_NAME)) == 0)
                    )
                {
                    ESP_LOGI(TAG, ">>> MATCH! Found radio server: %s", RADIO_DEVICE_NAME);
                    ESP_LOGI(TAG, "Radio RSSI: %d", event->disc.rssi);

                    // Save the address to RTC memory for fast reconnection after sleep
                    memcpy(&rtc_ble_cache.radio_mac_addr, &event->disc.addr, sizeof(ble_addr_t));
                    rtc_ble_cache.radio_mac_cached = true;

                    // Stop scanning
                    ble_gap_disc_cancel();
                    is_scanning = false;
                    is_connecting = true;

                    // Connect to the server
                    ESP_LOGI(TAG, "Connecting to radio server...");

                    // Set connection parameters balanced for latency and power
                    struct ble_gap_conn_params conn_params =
                    {
                        .scan_itvl = 0x0010,        // 10ms
                        .scan_window = 0x0010,      // 10ms
                        .itvl_min = 12,             // 15ms (12 * 1.25ms) - power optimized
                        .itvl_max = 12,             // 15ms (12 * 1.25ms) - keep same as min
                        .latency = 0,               // No latency
                        .supervision_timeout = 3200, // 32 seconds (3200 * 10ms)
                        .min_ce_len = 0,
                        .max_ce_len = 0,
                    };

                    rc = ble_gap_connect(own_addr_type, &rtc_ble_cache.radio_mac_addr,
                                        BLE_GAP_CONNECT_TIMEOUT_MS, &conn_params, nimble_gap_event, NULL);
                    if (rc != 0)
                    {
                        ESP_LOGE(TAG, "Failed to connect; rc=%d", rc);
                        is_connecting = false;
                    }
                }
            }
        break;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "Scan complete");
            is_scanning = false;
            if (!rtc_ble_cache.radio_mac_cached)
            {
                ESP_LOGW(TAG, "Radio server not found, retrying in 1 second...");
                // Retry after brief delay to allow radio to start advertising
                vTaskDelay(pdMS_TO_TICKS(1000));
                startScanAndConnect();
            }
        break;

        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "*** BLE_GAP_EVENT_CONNECT: status=%d ***", event->connect.status);
            if (event->connect.status == 0)
            {
                // Connection established - keep is_connecting=true until service discovery completes
                ESP_LOGI(TAG, "*** Connection status is 0, finding connection details ***");
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                ESP_LOGI(TAG, "*** ble_gap_conn_find returned rc=%d ***", rc);
                if (rc == 0)
                {
                    conn_handle = event->connect.conn_handle;
                    ESP_LOGI(TAG, "Connected to radio server, conn_handle: %d", conn_handle);

                    // Trigger LED flash state machine (non-blocking)
                    connection_flash_pending = true;
                    connection_flash_count = 0;
                    connection_flash_next_time = esp_timer_get_time() / 1000;

                    // Small delay to ensure GATT server is ready (50ms)
                    vTaskDelay(pdMS_TO_TICKS(50));

                    // Discover services
                    rc = ble_gattc_disc_all_svcs(conn_handle, gatt_disc_svc_cb, NULL);
                    if (rc != 0)
                    {
                        ESP_LOGE(TAG, "Failed to discover services; rc=%d", rc);
                        // Try to disconnect and reconnect
                        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Service discovery initiated");
                    }
                }
            }
            else
            {
                // Connection failed
                ESP_LOGE(TAG, "Connection failed; status=%d", event->connect.status);
                is_connecting = false;
                conn_handle = BLE_HS_CONN_HANDLE_NONE;

                // If this was a fast reconnect attempt that failed, try scanning
                if (rtc_ble_cache.radio_mac_cached && !is_scanning)
                {
                    ESP_LOGI(TAG, "Fast reconnect failed, falling back to scan");
                    rtc_ble_cache.radio_mac_cached = false;  // Invalidate cache, force scan
                    startScanAndConnect();
                }
            }
        break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected from radio server, reason: %d", event->disconnect.reason);
            power_mgr.ble_connected = false;
            is_connecting = false;
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            char_handle = 0;
            service_start_handle = 0;
            service_end_handle = 0;
            ble_fast_mode = false;  // Reset so next connection starts at normal interval

            // Release PM lock to allow light sleep again
            if (ble_pm_lock)
            {
                esp_pm_lock_release(ble_pm_lock);
                ESP_LOGI(TAG, "PM lock released - light sleep re-enabled");
            }

            ESP_LOGI(TAG, "BLE disconnected - will reconnect on next activity");

            if (system_events)
            {
                xEventGroupSetBits(system_events, BLE_DISCONNECTED_BIT);
            }
        break;

        case BLE_GAP_EVENT_CONN_UPDATE:
            if (event->conn_update.status == 0)
            {
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->conn_update.conn_handle, &desc) == 0)
                {
                    ESP_LOGI(TAG, "BLE interval confirmed: %.1fms",
                             desc.conn_itvl * 1.25f);
                }
            }
            else
            {
                // Peripheral rejected the update; reset so we can retry on next trigger
                ble_fast_mode = !ble_fast_mode;
                ESP_LOGW(TAG, "BLE interval update rejected: status=%d", event->conn_update.status);
            }
        break;

        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update: conn_handle=%d, mtu=%d",
                    event->mtu.conn_handle, event->mtu.value);
        break;
    }

    return 0;
}

///
/// @fn: gatt_disc_svc_cb
/// @details GATT service discovery callback
///
/// @param[in] conn_handle - connection handle
/// @param[in] error - pointer to error structure
/// @param[in] service - pointer to discovered service
/// @param[in] arg - user argument (unused)
///
/// @returns int - 0 on success
///
static int gatt_disc_svc_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_svc *service,
                            void *arg)
{
    if (error->status == 0)
    {
        // Check if this is our service (0x1234)
        if ( (service) && (ble_uuid_u16((ble_uuid_t *)&service->uuid) == RADIO_SERVICE_UUID16) )
        {
            service_start_handle = service->start_handle;
            service_end_handle = service->end_handle;
            ESP_LOGI(TAG, "Service found, handles: %d - %d",
                    service_start_handle, service_end_handle);

            // Discover characteristics
            int rc = ble_gattc_disc_all_chrs(conn_handle,
                                             service_start_handle,
                                             service_end_handle,
                                             gatt_disc_chr_cb, NULL);
            if (rc != 0)
            {
                ESP_LOGE(TAG, "Failed to start characteristic discovery; rc=%d", rc);
            }
        }
        return 0;
    }
    else if (error->status == BLE_HS_EDONE)
    {
        ESP_LOGI(TAG, "Service discovery complete");

        // Check if we found the service
        if (service_start_handle == 0)
        {
            ESP_LOGW(TAG, "Radio service (UUID 0x%04X) not found! Disconnecting...", RADIO_SERVICE_UUID16);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    ESP_LOGE(TAG, "Service discovery error: %d", error->status);

    // Disconnect on error
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

///
/// @fn: gatt_disc_chr_cb
/// @details GATT characteristic discovery callback
///
/// @param[in] conn_handle - connection handle
/// @param[in] error - pointer to error structure
/// @param[in] chr - pointer to discovered characteristic
/// @param[in] arg - user argument (unused)
///
/// @returns int - 0 on success
///
static int gatt_disc_chr_cb(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            const struct ble_gatt_chr *chr,
                            void *arg)
{
    if (error->status == 0)
    {
        // Check if this is our characteristic (0x5678)
        if (chr && ble_uuid_u16((ble_uuid_t *)&chr->uuid) == RADIO_CHAR_UUID16)
        {
            char_handle = chr->val_handle;
            ESP_LOGI(TAG, "Paddle characteristic found, handle: %d", char_handle);

            // Connection fully established - clear is_connecting flag
            is_connecting = false;
            power_mgr.ble_connected = true;

        #if USE_AUTO_BLE_SLEEP
            // Only acquire PM lock if using auto sleep (prevents interference)
            // Manual sleep control handles its own power management
            if (ble_pm_lock)
            {
                esp_pm_lock_acquire(ble_pm_lock);
                ESP_LOGI(TAG, "PM lock acquired - light sleep disabled");
            }
        #endif

            ESP_LOGI(TAG, "BLE connected - ready to send paddle events");

            if (system_events)
            {
                xEventGroupSetBits(system_events, BLE_CONNECTED_BIT);
            }
        }
        return 0;
    }
    else if (error->status == BLE_HS_EDONE)
    {
        ESP_LOGI(TAG, "Characteristic discovery complete");
        // Check if we found the characteristic
        if (char_handle == 0) {
            ESP_LOGW(TAG, "Radio characteristic (UUID 0x%04X) not found! Disconnecting...", RADIO_CHAR_UUID16);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);

    // Disconnect on error
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

///
/// @fn: startScanAndConnect
/// @details Initiates BLE scan to find and connect to radio server
///
/// @param[in] none
///
/// @returns none
///
static void startScanAndConnect(void)
{
    if (is_scanning || is_connecting || power_mgr.ble_connected)
    {
        ESP_LOGI(TAG, "Already scanning/connecting/connected, skipping");
        return;
    }

    ESP_LOGI(TAG, "Starting BLE scan for radio server");

    struct ble_gap_disc_params disc_params =
    {
        .filter_duplicates = 0,
        .passive = 0,           // Active scanning
        .itvl = 0x50,           // 50ms scan interval
        .window = 0x50,         // 50ms scan window (100% duty cycle for reliable discovery)
        .filter_policy = 0,     // Accept all
        .limited = 0,
    };

    int rc = ble_gap_disc(own_addr_type, BLE_GAP_DISCONNECT_TIMEOUT_MS, &disc_params,
                          nimble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to start scan; rc=%d", rc);
        return;
    }

    is_scanning = true;

    ESP_LOGI(TAG, "Scan started successfully");
}

///
/// @fn: tryDirectConnect
/// @details Attempts to connect directly to cached radio MAC address (skip scanning)
///          MAC address is persistent in RTC memory for fast reconnection after sleep
///
/// @param[in] none
///
/// @returns bool - true if connection initiated, false if cache invalid
///
static bool tryDirectConnect(void)
{
    if (!rtc_ble_cache.radio_mac_cached)
    {
        return false;  // No cached address, need to scan
    }

    ESP_LOGI(TAG, "Fast reconnect using RTC-cached MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             rtc_ble_cache.radio_mac_addr.val[5], rtc_ble_cache.radio_mac_addr.val[4],
             rtc_ble_cache.radio_mac_addr.val[3], rtc_ble_cache.radio_mac_addr.val[2],
             rtc_ble_cache.radio_mac_addr.val[1], rtc_ble_cache.radio_mac_addr.val[0]);

    is_connecting = true;

    // Use same connection parameters as scanning flow
    struct ble_gap_conn_params conn_params =
    {
        .scan_itvl = 0x0010,        // 10ms
        .scan_window = 0x0010,      // 10ms
        .itvl_min = 12,             // 15ms (power optimized)
        .itvl_max = 12,             // 15ms
        .latency = 0,
        .supervision_timeout = 3200, // 32 seconds
        .min_ce_len = 0,
        .max_ce_len = 0,
    };

    int rc = ble_gap_connect(own_addr_type, &rtc_ble_cache.radio_mac_addr,
                             BLE_GAP_CONNECT_TIMEOUT_MS, &conn_params,
                             nimble_gap_event, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Fast reconnect failed (rc=%d), will fall back to scanning", rc);
        is_connecting = false;
        rtc_ble_cache.radio_mac_cached = false;  // Invalidate cache, force scan
        return false;
    }

    ESP_LOGI(TAG, "Fast reconnect initiated");
    return true;
}

///
/// @fn: nimble_on_sync
/// @details Called when NimBLE stack is synchronized
///
/// @param[in] none
///
/// @returns none
///
static void nimble_on_sync(void)
{
    int rc;

    ESP_LOGI(TAG, "NimBLE host synchronized");

    // Make sure we have proper BLE address
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to ensure address; rc=%d", rc);
        return;
    }

    // Get our address type
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to infer address type; rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE address type: %d", own_addr_type);

    // Start scanning
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!tryDirectConnect())
    {
        // No cached address or direct connect failed - start scanning
        startScanAndConnect();
    }
}

///
/// @fn: nimble_on_reset
/// @details Called when NimBLE stack resets
///
/// @param[in] reason - reset reason code
///
/// @returns none
///
static void nimble_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE host reset; reason=%d", reason);
}

///
/// @fn: nimble_host_task
/// @details NimBLE host task
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
/// @fn: initBluetooth
/// @details Sets up BLE as GATT client using NimBLE stack
///
/// @param[in] none
///
/// @returns none
///
static void initBluetooth(void)
{
    ESP_LOGI(TAG, "Initializing Bluetooth (GATT Client - NimBLE)");

    // Ensure event group exists
    if (!system_events)
    {
        ESP_LOGE(TAG, "Event group not initialized - cannot start BLE");
        return;
    }

    // Initialize NimBLE
    ESP_ERROR_CHECK(nimble_port_init());

    // Set moderate TX power for power savings (+3dBm instead of +9dBm saves ~3-5mA)
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);
    ESP_LOGI(TAG, "BLE TX power set to +3 dBm (power optimized)");

    // Configure NimBLE host
    ble_hs_cfg.sync_cb = nimble_on_sync;
    ble_hs_cfg.reset_cb = nimble_on_reset;

    // Initialize GAP and GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Create PM lock to prevent light sleep during BLE connection
    esp_err_t pm_err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ble_lock", &ble_pm_lock);
    if (pm_err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to create PM lock: %d", pm_err);
    }

    // Start NimBLE host task
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "Bluetooth initialized successfully");
}

///
/// @fn: updateBleInterval
/// @details Requests a BLE connection parameter update to trade off latency
///          vs power based on detected CW speed.  Called with fast=true when
///          short press durations are observed (fast CW), and fast=false to
///          revert to the power-optimized normal interval after inactivity.
///
/// @param[in] fast - true: switch to 7.5ms interval; false: revert to 15ms
///
/// @returns none
///
static void updateBleInterval(bool fast)
{
    if (fast == ble_fast_mode || !power_mgr.ble_connected)
        return;

    struct ble_gap_upd_params params =
    {
        .itvl_min            = fast ? BLE_ITVL_FAST : BLE_ITVL_NORMAL,
        .itvl_max            = fast ? BLE_ITVL_FAST : BLE_ITVL_NORMAL,
        .latency             = 0,
        .supervision_timeout = 3200,
        .min_ce_len          = 0,
        .max_ce_len          = 0,
    };

    int rc = ble_gap_update_params(conn_handle, &params);
    if (rc == 0)
    {
        ble_fast_mode = fast;
        ESP_LOGI(TAG, "BLE interval → %s (%.1fms)",
                 fast ? "fast" : "normal",
                 (fast ? BLE_ITVL_FAST : BLE_ITVL_NORMAL) * 1.25f);
    }
    else
    {
        ESP_LOGW(TAG, "BLE interval update failed: rc=%d", rc);
    }
}

///
/// @fn: sendMorsePacket
/// @details Sends the paddle-transition messages to the radio-side ESP32
///
/// @param[in] paddle - which paddle (DIT or DAH)
/// @param[in] event_type - press or release event
/// @param[in] duration_ms - duration of paddle press in milliseconds
/// @param[in] timestamp - event timestamp for timing reconstruction
///
/// @returns none
///
static void sendMorsePacket(paddle_type_t paddle, event_type_t event_type, uint32_t duration_ms, uint64_t timestamp)
{
    if (!power_mgr.ble_connected || char_handle == 0)
    {
        ESP_LOGW(TAG, "sendMorsePacket: Not connected (ble_connected=%d, char_handle=%d)",
                 power_mgr.ble_connected, char_handle);
        return; // Not connected or characteristic not found
    }

    ble_morse_packet_t packet =
    {
        .paddle_type = paddle,
        .event_type = event_type,
        .mode = power_mgr.config.paddle_mode,
        .duration_ms = (duration_ms > 65535) ? 65535 : duration_ms,
        .timestamp = timestamp
    };

    const char *event_str = (event_type == EVENT_PRESS) ? "PRESS" : "RELEASE";
    ESP_LOGI(TAG, "sendMorsePacket: %s paddle=%d, duration=%"PRIu32"ms, conn_handle=%d, char_handle=%d, len=%zu",
             event_str, paddle, duration_ms, conn_handle, char_handle, sizeof(packet));

    int rc = ble_gattc_write_no_rsp_flat(conn_handle, char_handle,
                                        &packet, sizeof(packet));

    if (rc != 0)
    {
        ESP_LOGW(TAG, "Failed to send BLE packet: rc=%d", rc);
    }
    else
    {
        ESP_LOGI(TAG, "BLE packet sent successfully");
    }
}

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                     Process ULP Paddle-Transition Queue
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: processMorseEvents
/// @details When there are paddle-transition events in the shared ULP<>HP
///          queue, handle them.
///
/// @param[in] detection_time_ms : Timestamp value from queue item.
///
/// @returns boolean: True if activity detected. False else.
///
static bool processMorseEvents(uint64_t detection_time_ms)
{
    bool        activity_detected = false;
    uint32_t    initial_count = morse_queue->count;

    ESP_LOGI(TAG, "processMorseEvents called with %"PRIu32" events in queue", initial_count);

    // Warn if queue is getting dangerously full
    if (initial_count > (MAX_QUEUE_SIZE * 3 / 4))
    {
        ESP_LOGW(TAG, "Queue critically full: %"PRIu32"/%d events - processing urgently",
                 initial_count, MAX_QUEUE_SIZE);
    }
    else if (initial_count > (MAX_QUEUE_SIZE / 2))
    {
        ESP_LOGW(TAG, "Queue filling up: %"PRIu32"/%d events",
                 initial_count, MAX_QUEUE_SIZE);
    }

    // Process all queued events from ULP
    uint32_t sent_count = 0;
    while (morse_queue->count > 0)
    {
        // Memory barrier: ensure fresh read from ULP-written memory
        __sync_synchronize();

        // Capture current tail index
        uint32_t idx = morse_queue->tail;

        // Get event from queue - read all fields before updating pointers
        morse_event_t event = morse_queue->events[idx];

        // Memory barrier: ensure event data is fully read before updating tail/count
        __sync_synchronize();

        // Update tail pointer
        morse_queue->tail = (idx + 1) % MAX_QUEUE_SIZE;

        // Atomic decrement of count to prevent race with ULP's increment
        __sync_fetch_and_sub((volatile uint32_t*)&morse_queue->count, 1);

        // Control LED based on paddle state: ON for press, OFF for release
        if (event.event_type == EVENT_PRESS)
        {
            HP_LED_ON();
        }
        else
        {
            HP_LED_OFF();
        }

    #ifdef LOG_1_SECOND
        // Calculate ACTUAL queue processing latency (HP time when detected vs now)
        uint64_t now_ms = esp_timer_get_time() / 1000;
        int64_t queue_delay_ms = (int64_t)(now_ms - detection_time_ms);

        ESP_LOGI(TAG, "Event: ulp_ts=%"PRIu64" hp_detected=%"PRIu64" hp_sent=%"PRIu64" queue_delay=%"PRId64"ms depth=%"PRIu32,
                 event.timestamp, detection_time_ms, now_ms, queue_delay_ms, morse_queue->count + 1);
    #endif

        //
        // Adaptive BLE interval: short RELEASE durations → fast CW → needs lower latency
        //
        if (
               (event.event_type == EVENT_RELEASE) &&
               (event.duration_ms > 0) &&
               (event.duration_ms < FAST_CW_THRESHOLD_MS)
           )
        {
            last_fast_event_ms = esp_timer_get_time() / 1000;
            updateBleInterval(true);
        }

        // Send via BLE
        const char *event_str = (event.event_type == EVENT_PRESS) ? "PRESS" : "RELEASE";
        ESP_LOGI(TAG, "Sending event: %s paddle=%d, duration=%"PRIu32"ms", event_str, event.paddle, event.duration_ms);
        sendMorsePacket(event.paddle, event.event_type, event.duration_ms, event.timestamp);
        sent_count++;

        activity_detected = true;
    }

    if (sent_count > 0)
    {
        ESP_LOGI(TAG, "Sent %"PRIu32" events", sent_count);
    }

    return activity_detected;
}

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                        Configuration Button-Handling
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: buttonTask
/// @details FreeRTOS task that monitors the setup button and handles it.
///
/// @param[in] pvParameters - task parameters (unused)
///
/// @returns none
///
static void buttonTask(void *pvParameters)
{
    // Configure button GPIO
    gpio_config_t button_config =
    {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&button_config);

    bool        last_state = true;
    uint32_t    press_time = 0;

    while (1)
    {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        uint32_t current_time = esp_timer_get_time() / 1000;

        if (!current_state && last_state)
        {
            // Button pressed
            press_time = current_time;
        }
        else if (current_state && !last_state)
        {
            // Button released
            uint32_t duration = current_time - press_time;

            //
            // Button debounce
            //
            if (duration > 50)
            {
                if (duration < 1000)
                {
                    // Short press - cycle paddle mode
                    power_mgr.config.paddle_mode =
                        (power_mgr.config.paddle_mode + 1) % 4;
                    ESP_LOGI(TAG, "Paddle mode changed to %d", power_mgr.config.paddle_mode);
                }
                else
                {
                    // Long press - save config
                    saveConfigToNVS();
                    ESP_LOGI(TAG, "Configuration saved");
                }

                xEventGroupSetBits(system_events, BUTTON_PRESSED_BIT);
            }
        }

        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                        Power Management Functions
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: enablePowerManagementLogging
/// @details Enables detailed power management debug logging
///
/// @param[in] none
///
/// @returns none
///
static void enablePowerManagementLogging(void)
{
    // Enable detailed power management logging
    esp_log_level_set("pm", ESP_LOG_DEBUG);
    esp_log_level_set("sleep", ESP_LOG_DEBUG);
    esp_log_level_set("esp_pm", ESP_LOG_DEBUG);
    esp_log_level_set("clk", ESP_LOG_DEBUG);
    ESP_LOGI(TAG, "Power management debug logging enabled");
}

///
/// @fn: updateLastActivityTime
/// @details Updates the last activity timestamp and sets system state to active
///
/// @param[in] none
///
/// @returns none
///
static void updateLastActivityTime(void)
{
    power_mgr.last_activity_time = esp_timer_get_time() / 1000;
    power_mgr.current_state = SYSTEM_STATE_ACTIVE;
}

///
/// @fn: initAutomaticLightSleep
/// @details Sets up the HP core for automatic Light Sleep.  Allows
///          BLE connections to be maintained during light sleep
///          and automates entry and exit to/from Light Sleep
///
/// @param[in] none
///
/// @returns none
///
static void initAutomaticLightSleep(void)
{
    ESP_LOGI(TAG, "Configuring automatic light sleep");

    // Configure automatic power management
    esp_pm_config_t pm_config =
    {
        .max_freq_mhz = 160,        // Max CPU frequency
        .min_freq_mhz = 10,         // Min CPU frequency (will trigger light sleep)
        .light_sleep_enable = true   // Enable automatic light sleep
    };

    esp_err_t ret = esp_pm_configure(&pm_config);
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Automatic light sleep configured successfully");
}

#if !USE_AUTO_BLE_SLEEP
///
/// @fn: reinitConsoleUart
/// @details Reconfigures UART after wake from light sleep
///
/// @param[in] none
///
/// @returns none
///
static void reinitConsoleUart(void)
{
    // Reconfigure UART after wake from light sleep
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    #if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
    #elif SOC_UART_SUPPORT_XTAL_CLK
        .source_clk = UART_SCLK_XTAL,
    #endif
        .flags.backup_before_sleep = 1,
    };

    uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config);

    // Small delay for UART to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));
}


///
/// @fn: enterLightSleepMode
/// @details After some idle period, we will enter Light-Sleep mode,
///          keeping BLE connections alive.  Any paddle transitions
///          cause the ULP to wake us back up to full-power mode.
///
/// @param[in] none
///
/// @returns none
///
static void enterLightSleepMode(void)
{
    ESP_LOGI(TAG, "Entering light sleep mode");
    ESP_LOGI(TAG, "Inactive time: %llu ms",
             (esp_timer_get_time() / 1000) - power_mgr.last_activity_time);

    // Disconnect BLE before sleep - we'll reconnect on wake
    if (power_mgr.ble_connected)
    {
        ESP_LOGI(TAG, "Disconnecting BLE before sleep - will reconnect on wake");

        // Note: NimBLE will auto-disconnect when we sleep
        power_mgr.ble_connected = false;
    }

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    //
    // Enable LP Core (ULP) wakeup for paddle activity
    // AND timer wakeup to transition to deep sleep if no activity
    //
    esp_err_t wakeup_err = esp_sleep_enable_ulp_wakeup();
    ESP_LOGI(TAG, "LP Core wakeup enable result: %s", esp_err_to_name(wakeup_err));

    // Enable timer wakeup to transition to deep sleep after configured timeout
    uint64_t timer_us = (uint64_t)power_mgr.config.deep_sleep_timeout_ms * 1000ULL;
    esp_err_t timer_err = esp_sleep_enable_timer_wakeup(timer_us);
    ESP_LOGI(TAG, "Timer wakeup (%"PRIu32" ms) enable result: %s",
             power_mgr.config.deep_sleep_timeout_ms, esp_err_to_name(timer_err));

    ESP_LOGI(TAG, "About to enter light sleep... (queue depth: %lu, ULP wakeup_count: %lu)",
             (unsigned long)morse_queue->count, (unsigned long)ulp_wakeup_count);
    power_mgr.current_state = SYSTEM_STATE_LIGHT_SLEEP;

    // Suspend the queue processing task to allow light sleep to be entered
    if (queue_task_handle != NULL)
    {
        ESP_LOGI(TAG, "Suspending queue processing task");
        vTaskSuspend(queue_task_handle);
        ESP_LOGI(TAG, "Queue task suspended");
    }

    // Stop NimBLE to allow light sleep
    ESP_LOGI(TAG, "Stopping NimBLE stack...");
    int rc = nimble_port_stop();
    if (rc == 0)
    {
        ESP_LOGI(TAG, "NimBLE stopped, freeing...");
        nimble_port_deinit();
        ESP_LOGI(TAG, "NimBLE deinitialized");
    }
    else
    {
        ESP_LOGW(TAG, "NimBLE stop returned: %d", rc);
    }

    // Small delay to ensure tasks have time to clean up
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "*** CALLING esp_light_sleep_start() NOW ***");

    // Power down flash during sleep to save ~200-300µA
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);

    // Flush UART before sleep to ensure logs are visible
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

    //
    // Enter light sleep here. When we wake up, we should
    // resume at the next line
    //
    esp_err_t sleep_err = esp_light_sleep_start();

    //
    // Awake again.  Attempt console re-initialization
    //
    reinitConsoleUart();

    // Try ESP logging
    ESP_LOGI(TAG, "=== WOKE FROM LIGHT SLEEP (err=%d) ===", sleep_err);

    //
    // DON'T resume queue task yet - wait until BLE is connected
    // Otherwise it will try to send events before BLE is ready
    //
    if (sleep_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Light sleep FAILED with error: %s (0x%x)",
                 esp_err_to_name(sleep_err), sleep_err);
        power_mgr.current_state = SYSTEM_STATE_ACTIVE;

        // Try to reinit BLE even on error
        ESP_LOGI(TAG, "Reinitializing BLE after sleep error...");
        initBluetooth();
        return;
    }

    ESP_LOGI(TAG, "Successfully woke from light sleep!");
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Wakeup cause: %d", wakeup_cause);

    switch (wakeup_cause)
    {
        case ESP_SLEEP_WAKEUP_TIMER:
            // Timer expired - no paddle activity for extended period
            // Transition to deep sleep for maximum power savings
            ESP_LOGI(TAG, "Light sleep timer expired - entering deep sleep");
            ESP_LOGI(TAG, "Inactive for %"PRIu32" ms, transitioning to deep sleep mode",
                     power_mgr.config.deep_sleep_timeout_ms);

            // Don't reinitialize BLE - go directly to deep sleep
            enterDeepSleepMode();  // Doesn't return
        break;

        case ESP_SLEEP_WAKEUP_ULP:
            // LP Core (ULP) woke us - paddle activity detected
            ESP_LOGI(TAG, "Woke from light sleep due to paddle activity (LP Core)");
            updateLastActivityTime();
            power_mgr.current_state = SYSTEM_STATE_ACTIVE;
        break;

        case ESP_SLEEP_WAKEUP_BT:
            // BLE activity woke us (shouldn't normally happen, but handle it)
            ESP_LOGI(TAG, "Woke from light sleep due to BLE activity");
            updateLastActivityTime();
        break;

        default:
            ESP_LOGI(TAG, "Woke from light sleep, cause: %d", wakeup_cause);
            updateLastActivityTime();
        break;
    }

    // Reinitialize BLE FIRST before processing queued events
    ESP_LOGI(TAG, "Reinitializing BLE after wake...");
    initBluetooth();
    ESP_LOGI(TAG, "BLE reinitialized, starting connection...");

    // Wait for BLE connection to actually establish (up to 5 seconds)
    // Connection happens asynchronously - wait for the BLE_CONNECTED_BIT event
    ESP_LOGI(TAG, "Waiting for BLE connection...");
    EventBits_t bits = xEventGroupWaitBits(
        system_events,
        BLE_CONNECTED_BIT,
        pdFALSE,  // Don't clear the bit
        pdFALSE,  // Wait for the bit
        pdMS_TO_TICKS(5000)  // 5 second timeout
    );

    if (bits & BLE_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "BLE connected successfully!");
    }
    else
    {
        ESP_LOGW(TAG, "BLE connection timeout - will try to send anyway");
    }

    // NOW resume the queue processing task - BLE should be connected
    if (queue_task_handle != NULL)
    {
        ESP_LOGI(TAG, "Resuming queue processing task (BLE connected)");
        vTaskResume(queue_task_handle);
        ESP_LOGI(TAG, "Queue task resumed");
    }

    // NOW process any queued paddle events (only if woken by ULP)
    if (wakeup_cause == ESP_SLEEP_WAKEUP_ULP)
    {
        uint64_t detection_time = esp_timer_get_time() / 1000;
        __sync_synchronize();  // Force memory barrier
        uint32_t queued = morse_queue->count;
        ESP_LOGI(TAG, "Processing %"PRIu32" queued paddle events after ULP wakeup", queued);

        if (queued > 0)
        {
            while (processMorseEvents(detection_time))
            {
                xEventGroupSetBits(system_events, PADDLE_ACTIVITY_BIT);
            }
            ESP_LOGI(TAG, "Finished processing paddle events");
        }
    }

    ESP_LOGI(TAG, "System active and ready");
}

///
/// @fn: enterDeepSleepMode
/// @details After an idle period longer than the period for entering
///          Light Sleep mode, we will enter Deep Sleep mode.  Paddle
///          transitions will cause the ULP to wake us back up.  In that
///          case, we need to initialize almost everything, except the
///          ULP.
///
/// @param[in] none
///
/// @returns none (does not return - enters deep sleep)
///
static void enterDeepSleepMode(void)
{
    ESP_LOGI(TAG, "Entering deep sleep mode");

    // Gracefully disconnect BLE if connected
    if (power_mgr.ble_connected)
    {
        // Note: esp_ble_gap_disconnect needs device address, not connection ID
        // For simplicity, just disable BLE stack
        ESP_LOGI(TAG, "Disabling BLE for deep sleep");
        vTaskDelay(pdMS_TO_TICKS(100)); // Give time for any pending operations
    }

    // Shutdown BLE stack to save power
    esp_bt_controller_disable();

    //
    // Disable the low-power timer when entering deep-sleep, since it
    // wake us up again!
    //
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

    // Configure deep sleep wakeup
    esp_sleep_enable_ulp_wakeup();

    // Enter deep sleep - HP core resets on wakeup
    esp_deep_sleep_start();
}
#endif  // !USE_AUTO_BLE_SLEEP

#if !USE_AUTO_BLE_SLEEP

///
/// @fn: checkSleepTransitions
/// @details Based on our current state and idle timeout period,
///          we will transition into either Light or Deep Sleep
///          mode.
///
/// @param[in] none
///
/// @returns none
///
static void checkSleepTransitions(void)
{
    uint64_t            current_time = esp_timer_get_time() / 1000;
    uint64_t            inactive_time = current_time - power_mgr.last_activity_time;

    // Log every 5 seconds to track state
    static uint64_t last_log = 0;
    if (current_time - last_log >= 5000)
    {
        ESP_LOGI(TAG, "Sleep check: state=%d inactive=%llums timeout=%lums",
                 power_mgr.current_state, inactive_time,
                 (unsigned long)power_mgr.config.light_sleep_timeout_ms);
        last_log = current_time;
    }

    switch (power_mgr.current_state)
    {
        case SYSTEM_STATE_ACTIVE:
            if (inactive_time > power_mgr.config.light_sleep_timeout_ms)
            {
                ESP_LOGI(TAG, "*** Inactivity threshold reached - calling enterLightSleepMode() ***");
                enterLightSleepMode();
            }
        break;

        case SYSTEM_STATE_LIGHT_SLEEP:
            if (inactive_time > power_mgr.config.deep_sleep_timeout_ms)
            {
                enterDeepSleepMode();
            }
        break;

        case SYSTEM_STATE_DEEP_SLEEP:
            // Should never reach here
        break;
    }
}
#endif  // !USE_AUTO_BLE_SLEEP

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                        Setup/Initialization Functions
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: initMiscGPIO
/// @details Sets up any other IO
///
/// @param[in] none
///
/// @returns none
///
static void initMiscGPIO(void)
{
#if (USE_HP_LED==1)
    gpio_config_t ledConfig =
    {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&ledConfig);
    HP_LED_OFF();  // Initialize LED to off state
#endif
}

///
/// @fn: initUlpMorseHandler
/// @details If we have NOT already configured the ULP (which would be the case
///          immediately following power-up), we must configure it and get it
///          running.  We set up a 1 mS timeout that will wake the ULP for it
///          to sample the paddles.  When it sees transitions, it enqueues
///          messages that indicate which paddle was tapped and for how long.
///
/// @param[in] none
///
/// @returns none
///
static void initUlpMorseHandler(void)
{
    if (rtc_state.ulp_initialized)
    {
        ESP_LOGI(TAG, "ULP already initialized after deep sleep - reloading binary to reset state");
        // After deep sleep wakeup, fully reinitialize the ULP to ensure clean state
        // This reloads the binary and restarts execution from the beginning
    }
    else
    {
        ESP_LOGI(TAG, "Initializing ULP morse handler (first boot)");
    }

    // Always reload and restart the ULP to ensure proper state
    esp_err_t err = ulp_lp_core_load_binary(ulp_main_bin_start,
                                            (ulp_main_bin_end - ulp_main_bin_start));
    ESP_ERROR_CHECK(err);

    ulp_lp_core_cfg_t cfg =
    {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
        .lp_timer_sleep_duration_us = ULP_POLLING_INTERVAL,
    };

    err = ulp_lp_core_run(&cfg);
    ESP_ERROR_CHECK(err);

    rtc_state.ulp_initialized = true;
    ESP_LOGI(TAG, "ULP morse handler started/restarted");
}

///
/// @fn: handleDeepSleepWakeup
/// @details When we wake from Deep Sleep, we track how many times we
///          have been awakened (total). as well as how many times
///          we have been awakened from Deep Sleep.
///
/// @param[in] none
///
/// @returns none
///
static void handleDeepSleepWakeup(void)
{
    rtc_state.total_wakeups++;
    rtc_state.deep_sleep_count++;

    initMiscGPIO();

    ESP_LOGI(TAG, "Deep sleep wakeup #%"PRIu32" (total wakeups: %"PRIu32")",
             rtc_state.deep_sleep_count, rtc_state.total_wakeups);

    // Check what caused the wakeup
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Deep sleep wakeup cause: %d", wakeup_cause);

    if (wakeup_cause == ESP_SLEEP_WAKEUP_ULP)
    {
        ESP_LOGI(TAG, "Woke from deep sleep due to paddle activity (ULP)");
        // Force memory barrier to ensure fresh read from ULP
        __sync_synchronize();
        uint32_t queued = morse_queue->count;
        ESP_LOGI(TAG, "Queue has %"PRIu32" events after deep sleep wakeup", queued);
    }

    // ULP should still be running with preserved queue
    // No need to re-initialize ULP or reset queue

    // BLE connection lost during deep sleep - will need to reconnect
    power_mgr.ble_connected = false;
    char_handle = 0;
    service_start_handle = 0;
    service_end_handle = 0;
    conn_handle = BLE_HS_CONN_HANDLE_NONE;
    is_connecting = false;
    is_scanning = false;

    ESP_LOGI(TAG, "BLE will reconnect after deep sleep wakeup");
}

///
/// @fn: handleColdBoot
/// @details This will be called when we came up following
///          power-on.  Sets up stats, flags, etc.
///
/// @param[in] none
///
/// @returns none
///
static void handleColdBoot(void)
{
    ESP_LOGI(TAG, "Cold boot initialization");

    rtc_state.ulp_initialized = false;
    rtc_state.first_boot = true;
    rtc_state.deep_sleep_count = 0;
    rtc_state.total_wakeups = 0;

    initMiscGPIO();

    // Initialize shared queue
    memset(&ulp_morse_queue, 0, sizeof(ulp_morse_queue));
}

// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//                           Main Application Loop
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
// $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

///
/// @fn: mainOperatingLoop
/// @details Called from the entry-point function ("app_main"), this is responsible
///          for dispatching all the necessary application functions.
///
/// @param[in] none
///
/// @returns none (does not return - runs forever)
///

#define USE_OLD_SLEEP_TRANSITIONS    (1)
#if USE_OLD_SLEEP_TRANSITIONS

///
/// @fn: mainOperatingLoop
/// @details Main event loop for system operation and power management
///
/// @param[in] none
///
/// @returns none (does not return - runs forever)
///
static void mainOperatingLoop(void)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        EventBits_t events = xEventGroupWaitBits(
            system_events,
            PADDLE_ACTIVITY_BIT | BUTTON_PRESSED_BIT | BLE_CONNECTED_BIT | BLE_DISCONNECTED_BIT,
            pdTRUE,  // Clear bits on exit
            pdFALSE, // Wait for any bit
            pdMS_TO_TICKS(100)  // 100ms timeout
        );

        // Queue processing is now handled by dedicated queueProcessingTask()
        // No need to process here

        // Handle button events
        if (events & BUTTON_PRESSED_BIT)
        {
            updateLastActivityTime();
        }

        // Handle BLE connection events
        if (events & BLE_CONNECTED_BIT)
        {
            updateLastActivityTime();
            ESP_LOGI(TAG, "BLE connected - ready to send paddle events");
        }

        // Handle BLE disconnection events
        if (events & BLE_DISCONNECTED_BIT)
        {
            ESP_LOGI(TAG, "BLE disconnected - will reconnect on next activity");
            // Don't immediately reconnect - wait for activity
        }

    #if !USE_AUTO_BLE_SLEEP
        // Check for sleep transitions (manual sleep mode only)
        checkSleepTransitions();
    #endif

        // Maintain consistent timing
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}

#else

///
/// @fn: mainOperatingLoop
/// @details Main event loop for system operation and power management
///
/// @param[in] none
///
/// @returns none (does not return - runs forever)
///
static void mainOperatingLoop(void)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    while (1)
    {
        EventBits_t events = xEventGroupWaitBits(
            system_events,
            PADDLE_ACTIVITY_BIT | BUTTON_PRESSED_BIT | BLE_CONNECTED_BIT | BLE_DISCONNECTED_BIT,
            pdTRUE,  // Clear bits on exit
            pdFALSE, // Wait for any bit
            pdMS_TO_TICKS(20)  // 20ms timeout for responsive event processing
        );

        // Queue processing now handled by dedicated high-priority task

        // Handle button events
        if (events & BUTTON_PRESSED_BIT)
        {
            updateLastActivityTime();
        }

        // Handle BLE connection events
        if (events & BLE_CONNECTED_BIT)
        {
            updateLastActivityTime();
        }

        // Handle BLE disconnection - automatically reconnect
        if (events & BLE_DISCONNECTED_BIT)
        {
            ESP_LOGI(TAG, "Reconnecting to radio...");
            startScanAndConnect();
        }

        // Periodic connection check - ensure we connect if not already
        static uint64_t last_connection_attempt = 0;
        uint64_t now = esp_timer_get_time() / 1000;

        if (!power_mgr.ble_connected && !is_scanning && !is_connecting &&
            (now - last_connection_attempt > 5000))
        {
            ESP_LOGI(TAG, "Not connected, attempting to connect...");
            startScanAndConnect();
            last_connection_attempt = now;
        }

    #ifndef USE_AUTO_BLE_SLEEP
        // Check for sleep transitions (manual sleep mode only)
        checkSleepTransitions();
    #endif

        // Use 20ms delay for responsive paddle event processing
        // PM lock prevents light sleep during BLE connection anyway
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(20));
    }
}

#endif

///
/// @fn: queueProcessingTask
/// @details Event-driven FreeRTOS task for processing ULP morse queue.
///          Uses priority 4 (above NimBLE at 3) with ~10ms polling to allow IDLE to run.
///          Processes all events when detected for batch efficiency.
///
/// @param[in] pvParameters - FreeRTOS task parameter (unused)
///
/// @returns none (does not return - runs forever)
///
static void queueProcessingTask(void *pvParameters)
{
    ESP_LOGI(TAG, "Queue processing task started (event-driven, priority 4)");

    uint32_t poll_count = 0;
    uint64_t last_log_time = 0;

    while (1)
    {
        poll_count++;

        // Handle connection success LED flash state machine (non-blocking)
        uint64_t now = esp_timer_get_time() / 1000;

        // Revert BLE to power-saving normal interval after 1s without fast CW events
        if (ble_fast_mode && (now - last_fast_event_ms) > FAST_CW_REVERT_MS)
        {
            updateBleInterval(false);
        }

        if (connection_flash_pending)
        {
            if (now >= connection_flash_next_time)
            {
                if (connection_flash_count < 6)  // 3 flashes = 6 toggles (on, off, on, off, on, off)
                {
                    if (connection_flash_count % 2 == 0)
                        HP_LED_ON();
                    else
                        HP_LED_OFF();

                    connection_flash_count++;
                    connection_flash_next_time = now + 150;  // 150ms between toggles for visible flashing
                }
                else
                {
                    HP_LED_OFF();  // Ensure LED is off at end
                    connection_flash_pending = false;
                    ESP_LOGI(TAG, "Connection flash complete");
                }
            }
        }

        // Log ULP status and queue depth every 5 seconds for diagnostics
        if (now - last_log_time >= 5000)
        {
            uint32_t ulp_wakeups = ulp_wakeup_count;
            ESP_LOGI(TAG, "ULP wakeup_count=%"PRIu32" queue_depth=%"PRIu32" polls=%"PRIu32,
                     ulp_wakeups, morse_queue->count, poll_count);
            poll_count = 0;
            last_log_time = now;
        }

        // Force memory barrier to ensure fresh read from ULP-written memory
        __sync_synchronize();

        // Check if queue has events (event-driven behavior)
        uint32_t count = morse_queue->count;
        if (count > 0)
        {
            // Timestamp when we FIRST detect the event for accurate latency measurement
            uint64_t detection_time = esp_timer_get_time() / 1000;
        #ifdef LOG_1_SECOND
            ESP_LOGI(TAG, "*** Queue has %"PRIu32" events detected at hp_ts=%"PRIu64, count, detection_time);
        #endif
            // Process ALL queued events immediately
            while (processMorseEvents(detection_time))
            {
                updateLastActivityTime();
                xEventGroupSetBits(system_events, PADDLE_ACTIVITY_BIT);
            }
        }

        // Delay 1 tick (~10ms with 100Hz tick rate) to allow IDLE task to run
        vTaskDelay(1);
    }
}

///
/// @fn: app_main
/// @details Main application-entry point function.  Called following
///          power-up (or wakeup from Deep Sleep).  Does system-wide
///          setup (setting up FreeRTOS, deciding how to init, loading
///          config data from the NVS storage, etc.)
///
/// @param[in] none
///
/// @returns none
///
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if ENABLE_UART_LOGGING
    uart_config_t uart_config =
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,

    #if SOC_UART_SUPPORT_REF_TICK
        .source_clk = UART_SCLK_REF_TICK,
    #elif SOC_UART_SUPPORT_XTAL_CLK
        .source_clk = UART_SCLK_XTAL,
    #endif
        .flags.backup_before_sleep = 1,                 // CRITICAL: Auto backup/restore
    };

    // Reinstall UART driver with backup_before_sleep enabled for light sleep compatibility
    ESP_LOGI(TAG, "Configuring console UART for light sleep retention");

    // Uninstall the default console UART driver
    uart_driver_delete(CONFIG_ESP_CONSOLE_UART_NUM);

    // Reinstall with backup_before_sleep=1 flag
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));

    // Apply the configuration
    ESP_ERROR_CHECK(uart_param_config(CONFIG_ESP_CONSOLE_UART_NUM, &uart_config));

    // Reconfigure VFS to use the UART
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);

    ESP_LOGI(TAG, "Console UART reconfigured with retention support");
#else
    // UART disabled for power savings (~1-3mA)
    esp_log_level_set("*", ESP_LOG_NONE);  // Disable all logging
#endif

    // Create event group
    system_events = xEventGroupCreate();

    // Check reset reason and handle appropriately
    esp_reset_reason_t reset_reason = esp_reset_reason();

    if (reset_reason == ESP_RST_DEEPSLEEP)
    {
        handleDeepSleepWakeup();
    }
    else
    {
        handleColdBoot();
    }

    // Load configuration
    loadConfigFromNVS();

    //
    // Configure internal antenna for XIAO ESP32C6 BEFORE initializing ULP
    // GPIO 3 = LOW enables RF switch, GPIO 14 = LOW selects internal PCB antenna
    // This must happen BEFORE ULP init to avoid interfering with ULP GPIO setup
    //
    gpio_config_t antenna_conf =
    {
        .pin_bit_mask = (1ULL << GPIO_NUM_3) | (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&antenna_conf);
    gpio_set_level(GPIO_NUM_3, 0);   // LOW = enable RF switch
    gpio_set_level(GPIO_NUM_14, 0);  // LOW = internal PCB antenna
    ESP_LOGI(TAG, "Internal antenna configured (GPIO3=LOW, GPIO14=LOW)");

    // Initialize core systems
    initUlpMorseHandler();

    // Configure power management for reduced CPU frequency when idle
    esp_pm_config_t pm_config =
    {
        .max_freq_mhz = 80,     // Reduce from 160MHz (paddle processing doesn't need full speed)
        .min_freq_mhz = 40,     // Lower idle frequency
    #ifdef USE_AUTO_BLE_SLEEP
        .light_sleep_enable = true   // Automatic BLE-aware light sleep
    #else
        .light_sleep_enable = false  // Manual control via enterLightSleepMode()
    #endif
    };

    esp_err_t pm_err = esp_pm_configure(&pm_config);
    if (pm_err == ESP_OK)
    {
        ESP_LOGI(TAG, "Power management configured: 80MHz max, 40MHz min (saves ~3-5mA)");
    }
    else
    {
        ESP_LOGW(TAG, "Power management configuration failed: %s", esp_err_to_name(pm_err));
    }

    // Suppress verbose NimBLE GATT logging to reduce UART overhead
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    updateLastActivityTime();

    // Initialize peripherals and connectivity
    initBluetooth();

    // Start event-driven queue processing task (priority 6, well above NimBLE at 3)
    xTaskCreate(queueProcessingTask, "queue_task", 4096, NULL, QUEUE_TASK_PRIORITY, &queue_task_handle);

    // Start button handling task
    //xTaskCreate(buttonTask, "button_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Wireless paddle system ready");
    ESP_LOGI(TAG, "Light sleep timeout: %"PRIu32" ms", power_mgr.config.light_sleep_timeout_ms);
    ESP_LOGI(TAG, "Deep sleep timeout: %"PRIu32" ms", power_mgr.config.deep_sleep_timeout_ms);

    // Enter main operating loop
    mainOperatingLoop();
}
