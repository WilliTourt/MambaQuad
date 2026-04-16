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
        for (int i = 0; i < 5000; i++) {
            _dshot.send(100);
            this->delayUntil(pdMS_TO_TICKS(1));
        }

        for (int i = 0; i < 5000; i++) {
            _dshot.send(400);
            this->delayUntil(pdMS_TO_TICKS(1));
        }
    }
}
