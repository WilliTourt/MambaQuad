#include "BaroTask.h"

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
