#include "MagTask.h"
#include "DBGTask.h"

// MagCalResult g_magCal = {0};

MagTask::MagTask(I2C_HandleTypeDef *hi2c,
                 QMC5883P::QMC5883P_Mode mode, QMC5883P::QMC5883P_Spd spd,
                 FreeRTOS::Queue<MagData_t> &queue) :
                 Task(tskIDLE_PRIORITY + 3, 128, "Mag"),
                 _qmc5883p(hi2c, mode, spd),
                 _magQueue(queue) {}

bool MagTask::init() {
    auto status = _qmc5883p.begin();
    if (status != QMC5883P::QMC5883P_Status::OK) {
        HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
        return false;
    }
    return true;
}

void MagTask::taskFunction() {
    DBGQ.sendToBack((uint8_t*)"Magnetic sensor started to work", 0);

    // DBGQ.sendToBack((uint8_t*)"MagTask: Calibrating 45s, rotate sensor...", 0);
    // _qmc5883p.calibration(45);

    // // 存入全局变量，供蓝牙 MAGCAL 命令读取
    // g_magCal.offset_x = _qmc5883p.getOffsetX();
    // g_magCal.offset_y = _qmc5883p.getOffsetY();
    // g_magCal.offset_z = _qmc5883p.getOffsetZ();
    // g_magCal.scale_x  = _qmc5883p.getScaleX();
    // g_magCal.scale_y  = _qmc5883p.getScaleY();
    // g_magCal.scale_z  = _qmc5883p.getScaleZ();
    // g_magCal.valid = true;

    // DBGQ.sendToBack((uint8_t*)"MagTask: Calibration done. Use MAGCAL via BT to read.", 0);

    for (;;) {
        if (_qmc5883p.update() == QMC5883P::QMC5883P_Status::OK) {
            _magRawData.mx = _qmc5883p.getX();
            _magRawData.my = _qmc5883p.getY();
            _magRawData.mz = _qmc5883p.getZ();
            _magRawData.timestamp_ms = FreeRTOS::Kernel::getTickCount();
            _magQueue.sendToBack(_magRawData);
        }
        this->delay(pdMS_TO_TICKS(10));
    }
}
