#include "ControlTask.h"
#include "DBGTask.h"

volatile uint16_t ControlTask::motor_throttle[4] = {0, 0, 0, 0};

static ControlTask *instance = nullptr;

ControlTask::ControlTask(DShot &m1, DShot &m2, DShot &m3, DShot &m4,
                         FreeRTOS::Queue<ControlData_t> *queue) :
    Task(tskIDLE_PRIORITY + 4, 256, "CTRL"),
    _m1(m1), _m2(m2), _m3(m3), _m4(m4), _queue(queue) {
    instance = this;
}

void ControlTask::send() {
    static uint8_t idx = 0;
    switch (idx) {
        case 0: instance->_m1.send(motor_throttle[0]); break;
        case 1: instance->_m2.send(motor_throttle[1]); break;
        case 2: instance->_m3.send(motor_throttle[2]); break;
        case 3: instance->_m4.send(motor_throttle[3]); break;
    }
    idx = (idx + 1) & 0B011;
}

bool ControlTask::init() {
    bool ok = _m1.begin() && _m2.begin() && _m3.begin() && _m4.begin();
    return ok;
}

void ControlTask::_arm() {
    motor_throttle[0] = 48;
    motor_throttle[1] = 48;
    motor_throttle[2] = 48;
    motor_throttle[3] = 48;
    _enableSending();
    this->delayUntil(pdMS_TO_TICKS(2000));

    _armed = true;
    DBGQ.sendToBack((uint8_t*)"Motors armed.");
}

void ControlTask::_disarm() {
    motor_throttle[0] = 0;
    motor_throttle[1] = 0;
    motor_throttle[2] = 0;
    motor_throttle[3] = 0;
    this->delayUntil(pdMS_TO_TICKS(3000));
    _disableSending();

    _armed = false;
    DBGQ.sendToBack((uint8_t*)"Motors disarmed.");
}

void ControlTask::_enableSending() {
    HAL_TIM_Base_Start_IT(&htim2);
}

void ControlTask::_disableSending() {
    HAL_TIM_Base_Stop_IT(&htim2);
}

void ControlTask::taskFunction() {
    DBGQ.sendToBack((uint8_t*)"ControlTask started.");
    for (;;) {
        auto cmd = _queue->receive(portMAX_DELAY);
        if (cmd) {
            
            if (!_armed && cmd->armed) {
                DBGQ.sendToBack((uint8_t*)"ControlTask: Received arm command.");
                _arm();
            }
            
            if (_armed) {
                motor_throttle[0] = cmd->motor_throttle[0];
                motor_throttle[1] = cmd->motor_throttle[1];
                motor_throttle[2] = cmd->motor_throttle[2];
                motor_throttle[3] = cmd->motor_throttle[3];
            }
            
            if (_armed && !cmd->armed) {
                DBGQ.sendToBack((uint8_t*)"ControlTask: Received disarm command.");
                _disarm();
            }
        }
    }
}
