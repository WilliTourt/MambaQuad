#include "FreeRTOS.h"
#include "task.h"

extern "C" {
    /* 在单个源文件中定义静态内存并提供 FreeRTOS hook 实现 */
    static StaticTask_t idleTaskTCB;
    static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

    void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                         StackType_t **ppxIdleTaskStackBuffer,
                                         uint32_t *pulIdleTaskStackSize )
    {
        *ppxIdleTaskTCBBuffer = &idleTaskTCB;
        *ppxIdleTaskStackBuffer = idleTaskStack;
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }

    static StaticTask_t timerTaskTCB;
    static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

    void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                          StackType_t **ppxTimerTaskStackBuffer,
                                          uint32_t *pulTimerTaskStackSize )
    {
        *ppxTimerTaskTCBBuffer = &timerTaskTCB;
        *ppxTimerTaskStackBuffer = timerTaskStack;
        *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    }
}
