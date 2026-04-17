#include "MotorTask.h"

MotorTask::MotorTask(TIM_HandleTypeDef *htim, uint32_t channel, DShot::DShotType type) :
    Task(tskIDLE_PRIORITY + 3, 256, "MotorTask"),
    _dshot(htim, channel, type) {}

bool MotorTask::init() {
    return _dshot.begin();
}

void MotorTask::taskFunction() {
    _dshot.disarm();

    for (;;) {
        // test
        // for (int i = 900; i < 1100; i++) {
        //     _dshot.send(i);
        //     this->delayUntil(pdMS_TO_TICKS(1));
        // }

        // for (int i = 1100; i > 900; i--) {
        //     _dshot.send(i);
        //     this->delayUntil(pdMS_TO_TICKS(1));
        // }
        _dshot.send(1900);
        this->delayUntil(pdMS_TO_TICKS(1));
    }
}
