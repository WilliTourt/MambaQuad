#include "DBGTask.h"

FreeRTOS::Queue<uint8_t*> DBGQ(3);

#ifdef USB_AS_DEBUG
DBGTask::DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue,
                 FreeRTOS::Queue<uint8_t*> &generalDebugQueue) :
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
                 FreeRTOS::Queue<uint8_t*> &generalDebugQueue) :
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
        #if (DBG_ENABLE_GENERAL == 1)
            if (auto generalData = _generalDebugQueue.receive(0)) {
                _debug.log("%s%sGeneral:%s %s @%lums\r\n",
                           BOLD, COLOR_DARK_BLUE, CLR,
                           *generalData, xTaskGetTickCount());
                continue;
            }
        #endif

        #if (DBG_ENABLE_GPS == 1)
            if (auto gpsData = _gpsQueue.receive(pdMS_TO_TICKS(50))) {
                _debug.log("%s%sGPS:%s Lat=%.6f Lon=%.6f Alt=%.2f SatInView=%d SatInUse=%d @%lums\r\n",
                           BOLD, COLOR_DARK_GREEN, CLR,
                           gpsData->lat, gpsData->lon, gpsData->elv, gpsData->satInView, gpsData->satInUse, gpsData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_IMU == 1)
            if (auto imuData = _imuQueue.receive(0)) {
                _debug.log("%s%sIMU:%s aX=%.2f aY=%.2f aZ=%.2f gX=%.2f gY=%.2f gZ=%.2f @%lums\r\n",
                           BOLD, COLOR_DARK_YELLOW, CLR,
                           imuData->ax, imuData->ay, imuData->az,
                           imuData->gx, imuData->gy, imuData->gz,
                           imuData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_MAG == 1)
            if (auto magData = _magQueue.receive(0)) {
                _debug.log("%s%sMag:%s X=%.2fm Y=%.2fm Z=%.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_MAGENTA, CLR,
                           magData->mx, magData->my, magData->mz, magData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_BARO == 1)
            if (auto baroData = _baroQueue.receive(0)) {
                _debug.log("%s%sBaro:%s P=%.2fPa A=%.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_CYAN, CLR,
                           baroData->pressure_Pa, baroData->altitude_m, baroData->timestamp_ms);
                continue;
            }
        #endif

        this->delay(pdMS_TO_TICKS(7));
    }
}
