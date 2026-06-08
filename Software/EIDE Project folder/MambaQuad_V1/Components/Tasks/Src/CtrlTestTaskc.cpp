#include "CtrlTestTask.h"
#include "DBGTask.h"

CtrlTestTask::CtrlTestTask(FreeRTOS::Queue<ControlData_t> *to_ctrl_queue) :
    Task(tskIDLE_PRIORITY + 3, 256, "CtrlTest"),
    _to_ctrl_queue(to_ctrl_queue) {
    _data.armed = false;
    _data.motor_throttle[0] = 0;
    _data.motor_throttle[1] = 0;
    _data.motor_throttle[2] = 0;
    _data.motor_throttle[3] = 0;
}

void CtrlTestTask::taskFunction() {
    this->delay(pdMS_TO_TICKS(5000));
    DBGQ.sendToBack((uint8_t*)"CtrlTestTask: Test Started.");
    for (;;) {
        DBGQ.sendToBack((uint8_t*)"CtrlTestTask: Arming...");
        _data.armed = true;
        _to_ctrl_queue->sendToBack(_data);
        this->delay(pdMS_TO_TICKS(3000));

        DBGQ.sendToBack((uint8_t*)"CtrlTestTask: Testing four motors...");
        _data.motor_throttle[0] = 80;
        _data.motor_throttle[1] = 80;
        _data.motor_throttle[2] = 80;
        _data.motor_throttle[3] = 80;
        _to_ctrl_queue->sendToBack(_data);
        while (1) {
            // _to_ctrl_queue->sendToBack(_data);
            // this->delay(pdMS_TO_TICKS(50));
            this->suspend();
        }

        DBGQ.sendToBack((uint8_t*)"CtrlTestTask: Test done, sending zero throttle...");
        for (uint8_t i = 0; i < 4; i++) _data.motor_throttle[i] = 0;
        _to_ctrl_queue->sendToBack(_data);
        this->delay(pdMS_TO_TICKS(1000));

        DBGQ.sendToBack((uint8_t*)"CtrlTestTask: Disarming...");
        _data.armed = false;
        _to_ctrl_queue->sendToBack(_data);
        this->delay(pdMS_TO_TICKS(1000));
    }
}