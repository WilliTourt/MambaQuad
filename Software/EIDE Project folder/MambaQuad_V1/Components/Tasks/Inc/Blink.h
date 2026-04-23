#pragma once

#include "main.h"
#include <FreeRTOS/Task.hpp>
#include "DBGTask.h"

class BlinkTask : public FreeRTOS::Task {
    public:
        BlinkTask() : Task(tskIDLE_PRIORITY + 2, 128, "Blink") {}

    private:
        void taskFunction() override {
            for (;;) {
                DBGQ.sendToBack((uint8_t*)"Blinkyy~", 0);
                HAL_GPIO_WritePin(LED_LF_GPIO_Port, LED_LF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_RF_GPIO_Port, LED_RF_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_LR_GPIO_Port, LED_LR_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_RR_GPIO_Port, LED_RR_Pin, GPIO_PIN_SET);
                this->delayUntil(pdMS_TO_TICKS(100));
                HAL_GPIO_WritePin(LED_LF_GPIO_Port, LED_LF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_RF_GPIO_Port, LED_RF_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_LR_GPIO_Port, LED_LR_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_RR_GPIO_Port, LED_RR_Pin, GPIO_PIN_RESET);
                this->delayUntil(pdMS_TO_TICKS(900));
            }
        }
};
