#pragma once

#include "main.h"
#include <FreeRTOS/Task.hpp>

class BlinkTask : public FreeRTOS::Task {
    public:
        BlinkTask() : Task(tskIDLE_PRIORITY + 2, 128, "Blink") {}

    private:
        void taskFunction() override {
            for (;;) {
                HAL_GPIO_TogglePin(LED_LF_GPIO_Port, LED_LF_Pin);
                HAL_GPIO_TogglePin(LED_RF_GPIO_Port, LED_RF_Pin);
                HAL_GPIO_TogglePin(LED_LR_GPIO_Port, LED_LR_Pin);
                HAL_GPIO_TogglePin(LED_RR_GPIO_Port, LED_RR_Pin);
                this->delayUntil(pdMS_TO_TICKS(500));
            }
        }
};
