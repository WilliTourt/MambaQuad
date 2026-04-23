#include "MagTask.h"
#include "DBGTask.h"

MagTask::MagTask(I2C_HandleTypeDef *hi2c,
                 QMC5883P::QMC5883P_Mode mode, QMC5883P::QMC5883P_Spd spd,
                 FreeRTOS::Queue<MagData_t> &queue) :
                 Task(tskIDLE_PRIORITY + 3, 128, "Mag"),
                 _qmc5883p(hi2c, mode, spd),
                 _magQueue(queue) {}

bool MagTask::init() {
    auto status = _qmc5883p.begin();
    if (status != QMC5883P::QMC5883P_Status::OK) {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        return false;
    }
    return true;
}

void MagTask::taskFunction() {
    DBGQ.sendToBack((uint8_t*)"Magnetic sensor started to work", 0);
    for (;;) {
        if (_qmc5883p.update() == QMC5883P::QMC5883P_Status::OK) {
            _magRawData.mx = _qmc5883p.getX();
            _magRawData.my = _qmc5883p.getY();
            _magRawData.mz = _qmc5883p.getZ();
            _magRawData.timestamp_ms = FreeRTOS::Kernel::getTickCount();
            _magQueue.sendToBack(_magRawData);
        }
        this->delay(pdMS_TO_TICKS(3));
    }
}
