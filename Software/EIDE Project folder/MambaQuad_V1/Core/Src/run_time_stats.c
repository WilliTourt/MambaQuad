/* User implementation for FreeRTOS run-time statistics.
 * Uses Cortex-M DWT cycle counter. Kept in user code so CubeMX will not
 * remove it on project regeneration.
 */

#include "run_time_stats.h"
#include "stm32f4xx.h"

void vConfigureTimerForRunTimeStats( void )
{
    /* Enable trace and debug blocks if not already enabled */
    if( ( CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk ) == 0 )
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Reset the cycle counter */
    DWT->CYCCNT = 0;

    /* Enable the cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t ulGetRunTimeCounterValue( void )
{
    return ( uint32_t ) DWT->CYCCNT;
}
