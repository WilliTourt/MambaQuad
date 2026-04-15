#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "SerialTaskBase.h"
#include "ATGM336H.h"
#include "data_types.h"

class GPSSerialTask : public SerialTaskBase {
    public:
        GPSSerialTask(UART_HandleTypeDef *huart,
                      FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                      FreeRTOS::Queue<GPSData_t> &toGPSTaskQueue,
                      const char* taskName = "GPSSerialTask",
                      UBaseType_t priority = tskIDLE_PRIORITY + 2,
                      configSTACK_DEPTH_TYPE stackDepth = 1024);

    private:
        void rxProcess(uint8_t *data, uint16_t len) override;

        FreeRTOS::Queue<GPSData_t> &_toGPSTaskQueue;
        ATGM336H _gps;
};

class GPSTask : public FreeRTOS::Task {
    public:
        GPSTask(GPSSerialTask &serial,
                FreeRTOS::Queue<GPSData_t> &fromGPSSerialQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue);

    private:
        void taskFunction() override;

        GPSSerialTask &_serial;
        FreeRTOS::Queue<GPSData_t> &_fromGPSSerialQueue;
        FreeRTOS::Queue<GPSData_t> &_gpsQueue;
};