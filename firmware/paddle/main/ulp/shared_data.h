///=============================================================================
///
/// @copyright 2024, 2025, 2026 Todd D. Wade
///
/// @file
/// @brief
/// @details
///
///=============================================================================
// clang-format off

#pragma once
#include "esp_sleep.h"


#if defined(__cplusplus)
extern "C" {
#endif

#define USE_HP_LED                    (1)
#define ULP_USES_LED                  (0)  // HP core controls LED

// Debug flag: Set to 1 for short testing timeouts, 0 for production timeouts
#define DEBUG_SLEEP_TIMEOUTS          (0)

// Timeout values optimized for CW operation
#if (DEBUG_SLEEP_TIMEOUTS==1)
    // Debug timeouts for testing sleep transitions
    #define LIGHT_SLEEP_TIMEOUT_MS    (10000)   // 10 seconds for testing
    #define DEEP_SLEEP_TIMEOUT_MS     (20000)   // 20 seconds for testing
#else
    // Production timeouts for normal operation
    #define LIGHT_SLEEP_TIMEOUT_MS    (120000)  // 2 minutes
    #define DEEP_SLEEP_TIMEOUT_MS     (600000)  // 10 minutes
#endif

// shared_data.h - Include in both HP and ULP projects
#define MAX_QUEUE_SIZE              64      // Increased from 32 to reduce overflow risk
#define DEBOUNCE_TIME_US            50000   // 50ms debounce

// BLE Server configuration (radio-side device info)
#define RADIO_SERVICE_UUID16        0x1234
#define RADIO_CHAR_UUID16           0x5678
#define RADIO_DEVICE_NAME           "CW_Radio"

typedef enum
{
    PADDLE_DIT = 0,
    PADDLE_DAH = 1
} paddle_type_t;

typedef enum
{
    EVENT_PRESS = 0,
    EVENT_RELEASE = 1
} event_type_t;

typedef struct
{
    paddle_type_t       paddle;
    event_type_t        event_type;
    uint32_t            duration_ms;  // Only meaningful for RELEASE events
    uint64_t            timestamp;    // Changed from uint32_t to uint64_t to prevent overflow
} morse_event_t;

///
/// @struct morse_queue_t
/// @brief Shared queue between ULP and HP cores
/// @details SYNCHRONIZATION REQUIREMENTS:
///   - count: Must use atomic operations (__sync_fetch_and_add/sub)
///     since both ULP and HP cores modify it concurrently
///   - head: Only modified by ULP (producer)
///   - tail: Only modified by HP (consumer)
///   - events[]: ULP writes, HP reads - use memory barriers before/after
///     accessing to ensure data visibility across cores
///   - Always use __sync_synchronize() before waking HP core (ULP side)
///     and after reading count (HP side) to ensure memory coherency
///
typedef struct
{
    morse_event_t       events[MAX_QUEUE_SIZE];
    volatile uint32_t   head;     // Modified only by ULP (producer)
    volatile uint32_t   tail;     // Modified only by HP (consumer)
    volatile uint32_t   count;    // Modified by both - USE ATOMIC OPS
} morse_queue_t;


typedef enum
{
    DDB_STATE_FULL_POWER,
    DDB_STATE_LIGHT_SLEEP,
    DDB_STATE_DEEP_SLEEP
}ddbHPState;


#if defined(__cplusplus)
}
#endif

