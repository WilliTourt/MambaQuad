#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "data_types.h"

class CtrlTestTask : public FreeRTOS::Task {
    public:
        CtrlTestTask(FreeRTOS::Queue<ControlData_t> *to_ctrl_queue);

    private:
        void taskFunction() override;

        FreeRTOS::Queue<ControlData_t> *_to_ctrl_queue;
        ControlData_t _data;
};