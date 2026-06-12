#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "ElegantDebug.h"
#include "usbd_cdc_if.h"

#include "data_types.h"

#ifdef USB_AS_DEBUG_PORT
    #define USB_AS_DEBUG
#endif

#define DBG_ENABLE_IMU      0
#define DBG_ENABLE_MAG      0
#define DBG_ENABLE_BARO     0
#define DBG_ENABLE_GPS      0

#define DBG_ENABLE_GENERAL  1

extern FreeRTOS::Queue<uint8_t*> DBGQ;

class DBGTask : public FreeRTOS::Task {
    public:
    #ifdef USB_AS_DEBUG
        DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                FreeRTOS::Queue<MagData_t> &magQueue,
                FreeRTOS::Queue<BaroData_t> &baroQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue,
                FreeRTOS::Queue<uint8_t*> &generalDebugQueue,
                FreeRTOS::Queue<ControlData_t> &ctrlQueue,
                FreeRTOS::Queue<AttitudeData_t> &attQueue);
    #else
        DBGTask(UART_HandleTypeDef *huart,
                FreeRTOS::Queue<IMUData_t> &imuQueue,
                FreeRTOS::Queue<MagData_t> &magQueue,
                FreeRTOS::Queue<BaroData_t> &baroQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue,
                FreeRTOS::Queue<uint8_t*> &generalDebugQueue);
    #endif

    private:
        void taskFunction() override;
        void _processUsbCmd(const char* line);
        void _sendUsbResp(const char* msg);

        FreeRTOS::Queue<IMUData_t> &_imuQueue;
        FreeRTOS::Queue<MagData_t> &_magQueue;
        FreeRTOS::Queue<BaroData_t> &_baroQueue;
        FreeRTOS::Queue<GPSData_t> &_gpsQueue;
        FreeRTOS::Queue<uint8_t*> &_generalDebugQueue;
        
        #ifdef USB_AS_DEBUG
        FreeRTOS::Queue<ControlData_t> &_ctrlQueue;
        FreeRTOS::Queue<AttitudeData_t> &_attQueue;
        bool _armed = false;
        bool _telEnabled = false;
        TickType_t _lastTelTick = 0;
        AttitudeData_t _lastAtt = {};
        bool _hasAtt = false;
        uint16_t _motors[4] = {0, 0, 0, 0};
        #endif
        
        ElegantDebug _debug;
};
