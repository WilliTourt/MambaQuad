#include "IMUTask.h"

IMUTask* IMUTask::_instance = nullptr;

IMUTask::IMUTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                 uint16_t cs_pin, FreeRTOS::Queue<IMUData_t> &queue) :
                 Task(tskIDLE_PRIORITY + 3, 512, "IMU"),
                 _icm42688p(hspi, cs_port, cs_pin),
                 _imuQueue(queue) {}

bool IMUTask::init() {
    auto status = _icm42688p.begin();
    if (status != ICM42688P::ICM42688_StatusTypeDef::OK) {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        return false;
    }

    _icm42688p.general.enable_acc(ICM42688P::General::ACCEL_ODR::ACCEL_ODR_200HZ, 
                                  ICM42688P::General::ACCEL_FS::ACCEL_FS_2G,
                                  ICM42688P::General::FILTER_LEVEL::MEDIUM);
    _icm42688p.general.enable_gyro(ICM42688P::General::GYRO_ODR::GYRO_ODR_200HZ, 
                                   ICM42688P::General::GYRO_FS::GYRO_FS_500DPS,
                                   ICM42688P::General::FILTER_LEVEL::MEDIUM);

    // _icm42688p.Int.set_int1_pin_cfg(ICM42688P_INT_GPIO_Port, ICM42688P_INT_Pin, 
    //                                 ICM42688P::INT::LEVEL::LEVEL_ACTIVE_HIGH, 
    //                                 ICM42688P::INT::DRIVE::DRIVE_PUSH_PULL, 
    //                                 ICM42688P::INT::MODE::PULSE);

    // do *not* enable the data ready interrupt here.  enabling it before the
    // FreeRTOS scheduler has started (and before _instance is set) has been the
    // root cause of the “stuck high” behaviour: the first INT1 assertion is
    // lost and the pin never toggles again.  The interrupt will be enabled once
    // the task itself begins running (see taskFunction()).

    // _icm42688p.Int.register_event(ICM42688P::INT::IntEvent::DATA_RDY_INT,
    //                               ICM42688P::INT::INT_NUM::INT1, _drdyCallback, this);

    HAL_Delay(30);
    _calibrateBias(200);

    _instance = this;
    return true;
}

void IMUTask::taskFunction() {

    // // enable the DRDY interrupt only once the task is running and _instance
    // // has been initialized.  this avoids the startup race that previously
    // // required a debugger breakpoint to "kick" the line low.
    // _icm42688p.Int.set_int_source(ICM42688P::INT::INT_NUM::INT1,
    //                               ICM42688P::INT::IntEvent::DATA_RDY_INT,
    //                               true);

    // // clear any residual interrupt state inside the device and the MCU so the
    // // next rising edge is guaranteed to be seen.
    // (void)_icm42688p.Int.get_clr_int_state();
    // __HAL_GPIO_EXTI_CLEAR_IT(ICM42688P_INT_Pin);
    // HAL_NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

    // // if the line is already asserted we won't get a rising edge; in that case
    // // manually notify ourselves once so the task can clear the pending data.
    // if (HAL_GPIO_ReadPin(ICM42688P_INT_GPIO_Port, ICM42688P_INT_Pin) == GPIO_PIN_SET) {
    //     this->notifyGive();
    // }

    for (;;) {
        // FreeRTOS::Task::notifyTake(portMAX_DELAY, true);

        // _icm42688p.Int.get_clr_int_state();

        auto status = _icm42688p.general.read_data();
        if (status == ICM42688P::General::StatusTypeDef::OK) {
            _imuData_Raw.ax = _icm42688p.general.getAccelX();
            _imuData_Raw.ay = _icm42688p.general.getAccelY();
            _imuData_Raw.az = _icm42688p.general.getAccelZ();
            _imuData_Raw.gx = _icm42688p.general.getGyroX();
            _imuData_Raw.gy = _icm42688p.general.getGyroY();
            _imuData_Raw.gz = _icm42688p.general.getGyroZ();
            _imuData_Raw.timestamp_ms = FreeRTOS::Kernel::getTickCount();

            _imuData_Calib = _biasing(_imuData_Raw);

            _imuQueue.sendToBack(_imuData_Calib, 2);
            // if (!_imuQueue.sendToBack(_imuData_Calib, 2)) {
            //     HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
            // }
        }

        this->delayUntil(pdMS_TO_TICKS(5));
    }
}

void IMUTask::_calibrateBias(uint16_t samples) {
    bool isStable = false;
    IMUData_t prevData = {0};
    IMUData_t currData = {0};
    ICM42688P::General::StatusTypeDef status;

    while (!isStable) {

        status = _icm42688p.general.read_data();
        if (status == ICM42688P::General::StatusTypeDef::OK) {
            prevData.ax = _icm42688p.general.getAccelX();
            prevData.ay = _icm42688p.general.getAccelY();
            prevData.az = _icm42688p.general.getAccelZ();
            prevData.gx = _icm42688p.general.getGyroX();
            prevData.gy = _icm42688p.general.getGyroY();
            prevData.gz = _icm42688p.general.getGyroZ();
        }

        HAL_Delay(15);
        status = _icm42688p.general.read_data();
        if (status == ICM42688P::General::StatusTypeDef::OK) {
            currData.ax = _icm42688p.general.getAccelX();
            currData.ay = _icm42688p.general.getAccelY();
            currData.az = _icm42688p.general.getAccelZ();
            currData.gx = _icm42688p.general.getGyroX();
            currData.gy = _icm42688p.general.getGyroY();
            currData.gz = _icm42688p.general.getGyroZ();
        }

        if (abs(currData.ax - prevData.ax) < 0.01 &&
            abs(currData.ay - prevData.ay) < 0.01 &&
            abs(currData.az - prevData.az) < 0.01 &&
            abs(currData.gx - prevData.gx) < 0.1 &&
            abs(currData.gy - prevData.gy) < 0.1 &&
            abs(currData.gz - prevData.gz) < 0.1 && 
            currData.az > (g - 0.1f)) {
            isStable = true;
        }
    }

    IMUData_t sum = {0};

    for (uint16_t i = 0; i < samples; i++) {
        auto status = _icm42688p.general.read_data();
        if (status == ICM42688P::General::StatusTypeDef::OK) {
            sum.ax += _icm42688p.general.getAccelX();
            sum.ay += _icm42688p.general.getAccelY();
            // sum.az += _icm42688p.general.getAccelZ();
            sum.gx += _icm42688p.general.getGyroX();
            sum.gy += _icm42688p.general.getGyroY();
            sum.gz += _icm42688p.general.getGyroZ();
        }

        HAL_Delay(5);
    }

    _imuData_Bias.ax = sum.ax / samples;
    _imuData_Bias.ay = sum.ay / samples;
    // _imuData_Bias.az = sum.az / samples;
    // _imuData_Bias.az = _imuData_Bias.az - sqrtf((_imuData_Bias.ax * _imuData_Bias.ax +
                                                //  _imuData_Bias.ay * _imuData_Bias.ay +
                                                //  _imuData_Bias.az * _imuData_Bias.az));
    _imuData_Bias.az = 0.0f;

    _imuData_Bias.gx = sum.gx / samples;
    _imuData_Bias.gy = sum.gy / samples;
    _imuData_Bias.gz = sum.gz / samples;

    // _bias_collected = true;
}

IMUData_t IMUTask::_biasing(IMUData_t& data) {
    // if (!_bias_collected) { return data; }

    data.ax -= _imuData_Bias.ax;
    data.ay -= _imuData_Bias.ay;
    data.az -= _imuData_Bias.az;
    data.gx -= _imuData_Bias.gx;
    data.gy -= _imuData_Bias.gy;
    data.gz -= _imuData_Bias.gz;

    return data;
}



void IMUTask::intCallback_IRQ() {
    bool higherPriorityTaskWoken = false;
    if (_instance) {
        _instance->notifyGiveFromISR(higherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ICM42688P_INT_Pin) {
        IMUTask::intCallback_IRQ();
    }
}
