#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "ElegantDebug.h"

#include "data_types.h"

// BASE CLASS OF SERIAL TASKS
class SerialTaskBase : public FreeRTOS::Task {
    public:

        struct RxPacket {
            uint16_t length;
            uint8_t data[256];
        };

        SerialTaskBase(UART_HandleTypeDef *huart,
                       FreeRTOS::Queue<RxPacket> &rxQueue,
                       const char* taskName,
                       UBaseType_t priority = tskIDLE_PRIORITY + 2,
                       configSTACK_DEPTH_TYPE stackDepth = 512);

        bool init();

        bool send(uint8_t* data, uint16_t len);
        static void rxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

    protected:
        void taskFunction() override final;
        virtual void rxProcess(uint8_t *data, uint16_t len) = 0;

        UART_HandleTypeDef* _huart;
        FreeRTOS::Queue<RxPacket> &_rxQueue;

    private:
        static constexpr uint16_t RX_BUF_SIZE = 256;
        uint8_t _rxBuffer[RX_BUF_SIZE];

        static SerialTaskBase* _instances[5];
};
