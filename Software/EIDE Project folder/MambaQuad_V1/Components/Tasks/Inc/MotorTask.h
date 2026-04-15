#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "dshot.h"

class MotorTask : public FreeRTOS::Task {
    public:
        MotorTask(TIM_HandleTypeDef *htim, uint32_t channel, DShot::DShotType type);
        bool init();
        
    private:
        void taskFunction() override;

        DShot _dshot;
};
