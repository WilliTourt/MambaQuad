#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "SerialTaskBase.h"
#include "dx-lr01.h"

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
                 FreeRTOS::Queue<DXLR01::LoraMessage_t> &LoraQueue);

        bool init(uint8_t channel, uint8_t level, DXLR01::TransMode mode,
                  uint16_t address, uint8_t baud);

    private:
        void taskFunction() override;

        static bool loraSendCb(uint8_t* data, uint16_t len, void* userData) {
            auto* task = static_cast<LoraTask*>(userData);
            return task->_serial.send(data, len);
        }
        static bool loraReceiveCb(uint8_t* data, uint16_t& len, uint32_t timeout, void* userData) {
            auto* task = static_cast<LoraTask*>(userData);
            auto msg = task->_fromLoraSerialQueue.receive(pdMS_TO_TICKS(timeout));
            if (msg) {
                len = msg->length;
                memcpy(data, msg->data, len);
                return true;
            }
            return false;
        }

        LoraSerialTask &_serial;
        FreeRTOS::Queue<DXLR01::LoraMessage_t> &_fromLoraSerialQueue;
        FreeRTOS::Queue<DXLR01::LoraMessage_t> &_loraQueue;

        DXLR01 _lora;
};
