/* User run-time stats API
 * Implementation must live in user code (Core/Src/run_time_stats.c) so
 * CubeMX code generation will not remove it.
 */

#ifndef RUN_TIME_STATS_H
#define RUN_TIME_STATS_H

#include <stdint.h>

/* Initialize/enable a high resolution timer used for FreeRTOS run-time stats.
 * Implemented using the Cortex-M DWT cycle counter.
 */
void vConfigureTimerForRunTimeStats( void );

/* Return the current run-time counter value (32-bit). */
uint32_t ulGetRunTimeCounterValue( void );

#endif /* RUN_TIME_STATS_H */
