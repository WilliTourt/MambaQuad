#include "tasks.h"
#include <cmath>

extern "C" {
    /* 在单个源文件中定义静态内存并提供 FreeRTOS hook 实现 */
    static StaticTask_t idleTaskTCB;
    static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

    void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                         StackType_t **ppxIdleTaskStackBuffer,
                                         uint32_t *pulIdleTaskStackSize )
    {
        *ppxIdleTaskTCBBuffer = &idleTaskTCB;
        *ppxIdleTaskStackBuffer = idleTaskStack;
        *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    }

    static StaticTask_t timerTaskTCB;
    static StackType_t timerTaskStack[configTIMER_TASK_STACK_DEPTH];

    void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                          StackType_t **ppxTimerTaskStackBuffer,
                                          uint32_t *pulTimerTaskStackSize )
    {
        *ppxTimerTaskTCBBuffer = &timerTaskTCB;
        *ppxTimerTaskStackBuffer = timerTaskStack;
        *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    }
}

/*** DBGTask **********************************************************************/
#if (USB_AS_DEBUG == 1)
DBGTask::DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue) :
                 Task(tskIDLE_PRIORITY + 1, 256, "DBG"),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue) {}
#else
DBGTask::DBGTask(UART_HandleTypeDef *huart,
                 FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue) :
                 Task(tskIDLE_PRIORITY + 1, 256, "DBG"),
                 _huart(huart),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue) {}
#endif

void DBGTask::taskFunction() {
    for (;;) {
        #if (DBG_ENABLE_IMU == 1)
            auto imuData = _imuQueue.receive(portMAX_DELAY);
            if (imuData) {
                sprintf((char*)_dbg_buffer, "IMU: aX=%.2f aY=%.2f aZ=%.2f gX=%.2f gY=%.2f gZ=%.2f @%lums\r\n",
                        imuData->ax, imuData->ay, imuData->az,
                        imuData->gx, imuData->gy, imuData->gz,
                        imuData->timestamp_ms);

                #if (USB_AS_DEBUG == 1)
                    CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                #else
                    HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                #endif
            }
        #endif

        #if (DBG_ENABLE_MAG == 1)
            auto magData = _magQueue.receive(portMAX_DELAY);
            if (magData) {
                sprintf((char*)_dbg_buffer, "Mag: X=%.2fm Y=%.2fm Z=%.2fm @%lums\r\n",
                        magData->mx, magData->my, magData->mz, magData->timestamp_ms);
                
                #if (USB_AS_DEBUG == 1)
                    CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                #else
                    HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                #endif
            }
        #endif

        #if (DBG_ENABLE_BARO == 1)
            auto baroData = _baroQueue.receive(portMAX_DELAY);
            if (baroData) {
                sprintf((char*)_dbg_buffer, "Baro: P=%.2fPa A=%.2fm @%lums\r\n",
                        baroData->pressure_Pa, baroData->altitude_m, baroData->timestamp_ms);
                
                #if (USB_AS_DEBUG == 1)
                    CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                #else
                    HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                #endif
            }
        #endif
    }
}
/*** End of DBGTask ***************************************************************/








/*** IMUTask **********************************************************************/
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
            abs(currData.gz - prevData.gz) < 0.1) {
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

/*** End of IMUTask ***************************************************************/








/*** MagTask **********************************************************************/
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
/*** End of MagTask ***************************************************************/








/*** BaroTask *********************************************************************/
BaroTask::BaroTask(I2C_HandleTypeDef *hi2c,
                   ICP10111::ICP10111_MeasurementMode mode, FreeRTOS::Queue<BaroData_t> &queue) :
                   Task(tskIDLE_PRIORITY + 3, 128, "Baro"),
                   _icp10111(hi2c, mode),
                   _baroQueue(queue),
                   _altFilter(0.1f, 0.006f, 0.03f),
                   _pressureFilter(0.5f, 0.05f, 0.03f) {}

bool BaroTask::init() {
    auto status = _icp10111.begin();
    if (status != ICP10111::ICP10111_Status::OK) {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
        return false;
    }
    return true;
}

void BaroTask::taskFunction() {
    for (;;) {
        auto status = _icp10111.measure();
        if (status == ICP10111::ICP10111_Status::OK || status == ICP10111::ICP10111_Status::BUSY) {
			this->delayUntil(pdMS_TO_TICKS(30));
		} else if (status == ICP10111::ICP10111_Status::DRDY) {
			_icp10111.convertData();

			float raw_p = _icp10111.getPressure();
			float raw_alt = _icp10111.getAltitudeWithTemp();

            float filtered_p = _pressureFilter.update(raw_p);
            float filtered_alt = _altFilter.update(raw_alt);

            _baroData.pressure_Pa = filtered_p;
            _baroData.altitude_m = filtered_alt;
            _baroData.timestamp_ms = FreeRTOS::Kernel::getTickCount();

            _baroQueue.sendToBack(_baroData);

		} else {
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET);
		}
    }
}


BaroTask::AlphaBetaFilter::AlphaBetaFilter(float alpha, float beta, float dt) :
    _alpha(alpha), _beta(beta), _dt(dt) {}

float BaroTask::AlphaBetaFilter::update(float measurement) {
    if (!_initialized) {
        _init(measurement);
        return _x_hat;
    }

    _x_hat = _x_predict + _alpha * (measurement - _x_predict);
    _dx_hat = _dx_predict + _beta * ((measurement - _x_predict) / _dt);

    _x_predict = _x_hat + _dx_hat * _dt;
    _dx_predict = _dx_hat;

    return _x_hat;
}

void BaroTask::AlphaBetaFilter::_init(float initial_x, float initial_dx) {
    _x_hat = initial_x;
    _dx_hat = initial_dx;

    _x_predict = _x_hat + _dx_hat * _dt;
    _dx_predict = _dx_hat;

    _initialized = true;
}
/*** End of BaroTask **************************************************************/
