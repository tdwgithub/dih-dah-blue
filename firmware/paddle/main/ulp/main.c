///=============================================================================
///
/// @copyright 2024, 2025, 2026 Todd Wade
///
/// @file main.c
/// @brief
/// @details This is the ULP-core application.  It only does a few things:
///
///    1. Every time it is awakened, it reads the paddle input IO. If activated,
///       debounces them and if valid, adds an entry into a queue that is shared
///       between the ULP and HP cores.  This entry indicates which paddle(s)
///       made the transition and for how long.
///
///    2. When any paddle transition that has been qualified occurs and an
///       entry has been placed into the queue, the ULP wakes up the HP
///       core so that it can be transmitted to the radio-side ESP32.
///
///=============================================================================
// clang-format off

#include <stdint.h>
#include <stdbool.h>
#include "driver/rtc_io.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "shared_data.h"


morse_queue_t           morse_queue = {};
volatile uint32_t       wakeup_count = 0;

//
// Defines the pins used for the paddles
//
#define DIT_GPIO                LP_IO_NUM_0
#define DAH_GPIO                LP_IO_NUM_1


//
// LED-related definitions, etc.
//
#if (ULP_USES_LED==1)
    #define LED_GPIO            LP_IO_NUM_2
    static uint32_t             ledState = 0;

    #define ULP_LED_ON()        ulp_lp_core_gpio_set_level(LED_GPIO, 0)
    #define ULP_LED_OFF()       ulp_lp_core_gpio_set_level(LED_GPIO, 1)
#else
    #define ULP_LED_ON()
    #define ULP_LED_OFF()
#endif

// ULP local variables - marked volatile to prevent caching across sleep/wake cycles
static volatile uint32_t    dit_press_time = 0;
static volatile uint32_t    dah_press_time = 0;
static volatile bool        dit_pressed = false;
static volatile bool        dah_pressed = false;
static volatile uint64_t    msTicks = 0;
static volatile uint32_t    wakeupCause;


//
// Internal prototypes
//
static void debounce_and_process_gpio(int gpio_num, bool *pressed_state, uint32_t *press_time, paddle_type_t paddle_type);
static void configure_gpio_interrupts(void);


///
/// @fn: queue_is_full
/// @details Checks if the morse event queue is full
///
/// @param[in] none
///
/// @returns bool - true if queue is full, false otherwise
///
bool queue_is_full(void)
{
    return morse_queue.count >= MAX_QUEUE_SIZE;
}

///
/// @fn: queue_add_event
/// @details Adds a paddle event to the morse event queue
///
/// @param[in] paddle - which paddle (DIT or DAH)
/// @param[in] event_type - press or release event
/// @param[in] duration_ms - duration of the paddle press in milliseconds
///
/// @returns none
///
static void queue_add_event(paddle_type_t paddle, event_type_t event_type, uint32_t duration_ms)
{
    if (queue_is_full())
    {
        // Queue full - critical situation!
        // Wake HP core aggressively to drain queue
        ulp_lp_core_wakeup_main_processor();
        return; // Drop event to prevent corruption
    }

    // Capture current head index for this event
    uint32_t idx = morse_queue.head;

    // Add queue element - write all fields before updating pointers
    morse_queue.events[idx].paddle = paddle;
    morse_queue.events[idx].event_type = event_type;
    morse_queue.events[idx].duration_ms = duration_ms;
    morse_queue.events[idx].timestamp = msTicks;

    //
    // Memory barrier: ensure event data is written before updating head/count
    // This prevents HP core from seeing stale event data
    //
    __sync_synchronize();

    // Update head pointer
    morse_queue.head = (idx + 1) % MAX_QUEUE_SIZE;

    // Atomic increment of count to prevent race with HP core's decrement
    __sync_fetch_and_add(&morse_queue.count, 1);

    // Memory barrier: ensure all writes are visible before waking HP core
    __sync_synchronize();

    //
    // Wake HP core immediately for every event -
    // interrupt-driven processing for lowest latency
    //
    ulp_lp_core_wakeup_main_processor();
}

///
/// @fn: debounce_and_process_gpio
/// @details Debounces and processes GPIO paddle state changes
///
/// @param[in] gpio_num - GPIO pin number to monitor
/// @param[in,out] pressed_state - pointer to current press state
/// @param[in,out] press_time - pointer to press start time
/// @param[in] paddle_type - which paddle (DIT or DAH)
///
/// @returns none
///
static void debounce_and_process_gpio(int gpio_num, bool *pressed_state, uint32_t *press_time, paddle_type_t paddle_type)
{
    bool current_state = ulp_lp_core_gpio_get_level(gpio_num) == 0; // Active low

    if (current_state && !(*pressed_state))
    {
        // Press detected - send PRESS event immediately for low latency
        *pressed_state = true;
        *press_time = msTicks;

        // Send PRESS event immediately (duration=0 for press events)
        queue_add_event(paddle_type, EVENT_PRESS, 0);
    }
    else
    if (!current_state && *pressed_state)
    {
        // Release detected
        uint32_t duration_ms = msTicks - *press_time;

        // Only queue release if duration is reasonable (filter bounces)
        if (duration_ms > 5)  // Minimum 5ms to filter bounces
        {
            *pressed_state = false;

            // Send RELEASE event with duration
            queue_add_event(paddle_type, EVENT_RELEASE, duration_ms);
        }
    }
}

///
/// @fn: configure_gpio
/// @details Sets up the GPIO pins that we will use
///          to read the paddles.
///
/// @param[in] none
///
/// @returns none
///
static void configure_gpio(void)
{
    // Configure DIT paddle GPIO - using external 100kΩ pull-up to 3.3V
    ulp_lp_core_gpio_init(DIT_GPIO);
    ulp_lp_core_gpio_input_enable(DIT_GPIO);
    ulp_lp_core_gpio_pullup_disable(DIT_GPIO);   // External pull-up used (saves ~40µA when pressed)
    ulp_lp_core_gpio_pulldown_disable(DIT_GPIO);

    // Configure DAH paddle GPIO - using external 100kΩ pull-up to 3.3V
    ulp_lp_core_gpio_init(DAH_GPIO);
    ulp_lp_core_gpio_input_enable(DAH_GPIO);
    ulp_lp_core_gpio_pullup_disable(DAH_GPIO);   // External pull-up used (saves ~40µA when pressed)
    ulp_lp_core_gpio_pulldown_disable(DAH_GPIO);

#if (ULP_USES_LED==1)
    ulp_lp_core_gpio_init(LED_GPIO);
    ulp_lp_core_gpio_output_enable(LED_GPIO);
    ulp_lp_core_gpio_pullup_disable(LED_GPIO);
    ulp_lp_core_gpio_pulldown_disable(LED_GPIO);
    ULP_LED_OFF();
#endif
}

///
/// @fn: main
/// @details ULP main entry point. Monitors paddle GPIOs and queues events.
///
/// @param[in] none
///
/// @returns none
///
void main(void)
{
    wakeup_count++;

    // Configure GPIOs on first run
    if (wakeup_count == 1)
    {
        configure_gpio();
    }

    // Decide why we were awakened
    wakeupCause = ulp_lp_core_get_wakeup_cause();

    //
    // We keep a monotonically-increasing "tick" value
    // that represents the # of milliseconds since startup.
    // This allows us to put timing into each paddle transition
    // record that is added to the queue.
    //
    if (wakeupCause & ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER)
    {
        ++msTicks;  // 1 mS per tick
    }

    // Process paddles
    debounce_and_process_gpio(DIT_GPIO, &dit_pressed, &dit_press_time, PADDLE_DIT);
    debounce_and_process_gpio(DAH_GPIO, &dah_pressed, &dah_press_time, PADDLE_DAH);

    // Exiting causes the ULP to go back to sleep
    return;
}
