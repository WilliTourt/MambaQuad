#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "SerialTaskBase.h"
#include "dx-lr01.h"

#include "DBGTask.h"

#include "data_types.h"

class LoraSerialTask : public SerialTaskBase {
    public:
        LoraSerialTask(UART_HandleTypeDef *huart,
                       FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                       FreeRTOS::Queue<DXLR01::LoraMessage_t> &toLoraTaskQueue,
                       const char* taskName = "LoraSerialTask",
                       UBaseType_t priority = tskIDLE_PRIORITY + 2,
                       configSTACK_DEPTH_TYPE stackDepth = 512);
        
    private:
        void rxProcess(uint8_t *data, uint16_t len) override;

        FreeRTOS::Queue<DXLR01::LoraMessage_t> &_toLoraTaskQueue;
};

class LoraTask : public FreeRTOS::Task {
    public:

        LoraTask(LoraSerialTask &serial,
                 FreeRTOS::Queue<DXLR01::LoraMessage_t> &fromLoraSerialQueue,
                 FreeRTOS::Queue<DXLR01::LoraMessage_t> &LoraQueue,
                 FreeRTOS::Queue<ControlData_t> &ctrlQueue,
                 FreeRTOS::Queue<AttitudeData_t> &attQueue);

        bool init(uint8_t channel, uint8_t level, DXLR01::TransMode mode,
                  uint16_t address, uint8_t baud);

    private:
        void taskFunction() override;
        void parseCommand(const char* cmd);
        void _sendTelemetry();

        LoraSerialTask &_serial;
        FreeRTOS::Queue<DXLR01::LoraMessage_t> &_fromLoraSerialQueue;
        FreeRTOS::Queue<DXLR01::LoraMessage_t> &_loraQueue;
        FreeRTOS::Queue<ControlData_t> &_ctrlQueue;
        FreeRTOS::Queue<AttitudeData_t> &_attQueue;

        AttitudeData_t _lastAtt;
        bool _hasAtt = false;
        bool _telEnabled = true;
        uint32_t _lastTelTick = 0;

        // 状态
        bool _armed = false;
        uint16_t _motors[4] = {0, 0, 0, 0};

        DXLR01 _lora;
};
