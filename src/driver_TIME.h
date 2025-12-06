#pragma once

#include <stdbool.h>
#include <stdint.h>

/*! @brief Get the ADC trigger period
 *  @return trigger period in microsceconds
 */
uint16_t timerADCPeriod(void);

/*! @brief  Blocking delay. Use with caution.
 *  @param [in] delay : period in ms
 *  @return true if successful, false otherwise
 */
bool timerDelay_ms(uint16_t delay);

/*! @brief  Blocking delay. Use with caution.
 *  @param [in] delay : period in us
 *  @return true if successful, false otherwise
 */
bool timerDelay_us(uint32_t delay);

/*! @brief  Start the elapsed time counter at 1 us resolution.
 *  @return true if successful, false otherwise
 */
bool timerElapsedStart(void);

/*! @brief  End the elapsed time counter. Returns the elapsed time in us.
 */
uint32_t timerElapsedStop(void);

/*! @brief Returns the current microsecond count value
 */
uint32_t timerMicros(void);

/*! @brief Returns the time delta between microseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return time delta in microseconds
 */
uint32_t timerMicrosDelta(const uint32_t prevMicros);

/*! @brief Returns the current millisecond count value
 */
uint32_t timerMillis(void);

/*! @brief Returns the time delta between milliseconds, accounting for wrap
 *  @param [in] prevMicros : previous count value
 *  @return time delta in milliseconds
 */
uint32_t timerMillisDelta(const uint32_t prevMillis);

/*! @brief  Sets up the system timer units */
void timerSetup(void);

/*! @brief Returns the uptime in seconds of the whole system
 *  @return uptime in seconds
 */
uint32_t timerUptime(void);

/*! @brief Increment the uptime counter */
void timerUptimeIncr(void);

/* Timed callback queue system */

/*! @brief Callback function type for timed events
 *  @note Callbacks execute in MAIN LOOP context (not ISR)
 *        They can perform blocking operations safely
 */
typedef void (*TimerCallback_t)(void);

/*! @brief Schedule a callback to be executed after a specified delay
 *  @param [in] callback : function to call
 *  @param [in] delay_us : delay in microseconds
 *  @return true if scheduled successfully, false if queue is full
 */
bool timerScheduleCallback(TimerCallback_t callback, uint32_t delay_us);

/*! @brief Cancel a pending callback
 *  @param [in] callback : function to cancel
 *  @return true if found and cancelled, false otherwise
 */
bool timerCancelCallback(TimerCallback_t callback);

/*! @brief Check if a callback is pending
 *  @param [in] callback : function to check
 *  @return true if pending, false otherwise
 */
bool timerCallbackPending(TimerCallback_t callback);

/*! @brief Execute pending callbacks (call from main loop)
 *  @details Executes all callbacks that were triggered by timer interrupts
 *           Must be called regularly from main loop
 */
void timerProcessPendingCallbacks(void);
