#include "DBGTask.h"

#ifdef USB_AS_DEBUG
DBGTask::DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue,
                 FreeRTOS::Queue<uint8_t> &generalDebugQueue) :
                 Task(tskIDLE_PRIORITY + 1, 512, "DBG"),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue),
                 _gpsQueue(gpsQueue),
                 _generalDebugQueue(generalDebugQueue),
                 _debug(true, true, true) {}
#else
DBGTask::DBGTask(UART_HandleTypeDef *huart,
                 FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue,
                 FreeRTOS::Queue<uint8_t> &generalDebugQueue) :
                 Task(tskIDLE_PRIORITY + 1, 512, "DBG"),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue),
                 _gpsQueue(gpsQueue),
                 _generalDebugQueue(generalDebugQueue),
                 _debug(huart, true, true, true) {}
#endif

void DBGTask::taskFunction() {
    for (;;) {
        #if (DBG_ENABLE_IMU == 1)
            auto imuData = _imuQueue.receive(portMAX_DELAY);
            if (imuData) {
                _debug.log("%s%sIMU:%s aX=%.2f aY=%.2f aZ=%.2f gX=%.2f gY=%.2f gZ=%.2f @%lums\r\n",
                           BOLD, COLOR_DARK_YELLOW, CLR, 
                           imuData->ax, imuData->ay, imuData->az,
                           imuData->gx, imuData->gy, imuData->gz,
                           imuData->timestamp_ms);
            }
        #endif

        #if (DBG_ENABLE_MAG == 1)
            auto magData = _magQueue.receive(portMAX_DELAY);
            if (magData) {
                _debug.log("%s%sMag:%s X=%.2fm Y=%.2fm Z=%.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_MAGENTA, CLR, 
                           magData->mx, magData->my, magData->mz, magData->timestamp_ms);
            }
        #endif

        #if (DBG_ENABLE_BARO == 1)
            auto baroData = _baroQueue.receive(portMAX_DELAY);
            if (baroData) {
                _debug.log("%s%sBaro:%s P=%.2fPa A=%.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_CYAN, CLR, 
                           baroData->pressure_Pa, baroData->altitude_m, baroData->timestamp_ms);
            }
        #endif

        #if (DBG_ENABLE_GPS == 1)
            auto gpsData = _gpsQueue.receive(portMAX_DELAY);
            if (gpsData) {
                _debug.log("%s%sGPS:%s Lat=%.6f Lon=%.6f Alt=%.2f SatInView=%d SatInUse=%d @%lums\r\n",
                           BOLD, COLOR_DARK_GREEN, CLR,
                           gpsData->lat, gpsData->lon, gpsData->elv, gpsData->satInView, gpsData->satInUse, gpsData->timestamp_ms);
            }
        #endif

        #if (DBG_ENABLE_GENERAL == 1)
            auto generalData = _generalDebugQueue.receive(portMAX_DELAY);
            if (generalData) {
                _debug.log("%s%sGeneral:%s %s @%lums\r\n",
                           BOLD, COLOR_DARK_BLUE, CLR,
                           *generalData, xTaskGetTickCount());
            }
        #endif
    }
}
