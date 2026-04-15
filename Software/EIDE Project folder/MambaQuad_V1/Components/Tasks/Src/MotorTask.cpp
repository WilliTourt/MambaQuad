#include "MotorTask.h"

MotorTask::MotorTask(TIM_HandleTypeDef *htim, uint32_t channel, DShot::DShotType type) :
    Task(tskIDLE_PRIORITY + 3, 256, "MotorTask"),
    _dshot(htim, channel, type) {}

bool MotorTask::init() {
    return _dshot.begin();
}

void MotorTask::taskFunction() {
    for (;;) {
        // test
        _dshot.send(0);
        this->delayUntil(pdMS_TO_TICKS(1000));
        _dshot.send(100);
        this->delayUntil(pdMS_TO_TICKS(1000));
        _dshot.send(500);
        this->delayUntil(pdMS_TO_TICKS(1000));
        _dshot.send(1000);
        this->delayUntil(pdMS_TO_TICKS(1000));
    }
}
