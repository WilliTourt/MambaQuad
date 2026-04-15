#include "GPSTask.h"

GPSTask::GPSTask(GPSSerialTask &serial,
                 FreeRTOS::Queue<GPSData_t> &fromGPSSerialQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue) :
    Task(tskIDLE_PRIORITY + 2, 512, "GPS"),
    _serial(serial),
    _fromGPSSerialQueue(fromGPSSerialQueue),
    _gpsQueue(gpsQueue) {}

void GPSTask::taskFunction() {
    for (;;) {
        auto gps_data_opt = _fromGPSSerialQueue.receive(portMAX_DELAY);
        if (gps_data_opt) {
            GPSData_t gps_data = *gps_data_opt;

            // convert from DDMM.MMMM format to decimal degrees
            gps_data.lat = (gps_data.lat >= 0 ? 1 : -1) * ( (int)(fabs(gps_data.lat)) / 100 + (int)(fabs(gps_data.lat)) % 100 / 60.0f + (fabs(gps_data.lat) - (int)(fabs(gps_data.lat))) / 60.0f );
            gps_data.lon = (gps_data.lon >= 0 ? 1 : -1) * ( (int)(fabs(gps_data.lon)) / 100 + (int)(fabs(gps_data.lon)) % 100 / 60.0f + (fabs(gps_data.lon) - (int)(fabs(gps_data.lon))) / 60.0f );

            _gpsQueue.sendToBack(gps_data);
        }
    }
}



GPSSerialTask::GPSSerialTask(UART_HandleTypeDef *huart,
                             FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                             FreeRTOS::Queue<GPSData_t> &toGPSTaskQueue,
                             const char* taskName,
                             UBaseType_t priority,
                             configSTACK_DEPTH_TYPE stackDepth) :
    SerialTaskBase(huart, rxQueue, taskName, priority, stackDepth),
    _toGPSTaskQueue(toGPSTaskQueue) {}

void GPSSerialTask::rxProcess(uint8_t *data, uint16_t len) {
    GPSData_t gps_data;

    if (_gps.update(data)) {
        gps_data.lat = _gps.get_lat();
        gps_data.lon = _gps.get_lon();
        gps_data.elv = _gps.get_elv();
        gps_data.speed = _gps.get_speed();
        gps_data.dir = _gps.get_direction();

        gps_data.year = _gps.get_UTC_year();
        gps_data.month = _gps.get_UTC_month();
        gps_data.day = _gps.get_UTC_day();
        gps_data.hour = _gps.get_UTC_hour();
        gps_data.min = _gps.get_UTC_minute();
        gps_data.sec = _gps.get_UTC_second();

        gps_data.satInUse = _gps.get_satinfo_inuse();
        gps_data.satInView = _gps.get_satinfo_inview();

        gps_data.timestamp_ms = FreeRTOS::Kernel::getTickCount();
        _toGPSTaskQueue.sendToBack(gps_data);
    }
}
