#include "LoraTask.h"
#include "MagTask.h"
#include "ControlTask.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

#define RAD2DEG(r) ((r) * 57.29578f)

LoraTask::LoraTask(LoraSerialTask &serial,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &fromLoraSerialQueue,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &loraQueue,
    FreeRTOS::Queue<ControlData_t> &ctrlQueue,
    FreeRTOS::Queue<AttitudeData_t> &attQueue) :
    Task(tskIDLE_PRIORITY + 2, 512, "LoRa"),
    _serial(serial),
    _fromLoraSerialQueue(fromLoraSerialQueue),
    _loraQueue(loraQueue),
    _ctrlQueue(ctrlQueue),
    _attQueue(attQueue) {
    _lastAtt = {};
}

bool LoraTask::init(uint8_t channel, uint8_t level, DXLR01::TransMode mode,
                    uint16_t address, uint8_t baud) {
    this->delay(pdMS_TO_TICKS(200));
    return true;
}

void LoraTask::parseCommand(const char* cmd) {
    static char resp[256];
    char cmdCopy[256];
    strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
    cmdCopy[sizeof(cmdCopy) - 1] = '\0';

    char* tok = strtok(cmdCopy, " \r\n");
    if (!tok) return;

    for (char* p = tok; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

    if (strcmp(tok, "MOTOR") == 0 || strcmp(tok, "M") == 0) {
        char* ns = strtok(nullptr, " \r\n");
        char* vs = strtok(nullptr, " \r\n");
        if (!ns || !vs) {
            _serial.send((uint8_t*)"ERR: MOTOR <n|ALL> <0-1000>\r\n", 29);
            return;
        }
        uint16_t speed = (uint16_t)strtoul(vs, nullptr, 10);
        if (speed > 1000) speed = 1000;

        if (strcmp(ns, "ALL") == 0) {
            for (int i = 0; i < 4; i++) _motors[i] = speed;
            snprintf(resp, sizeof(resp), "OK: MOTOR ALL=%u\r\n", speed);
        } else {
            int n = strtoul(ns, nullptr, 10);
            if (n < 0 || n > 3) {
                _serial.send((uint8_t*)"ERR: motor index 0-3\r\n", 22);
                return;
            }
            _motors[n] = speed;
            snprintf(resp, sizeof(resp), "OK: MOTOR %d=%u\r\n", n, speed);
        }
        _serial.send((uint8_t*)resp, strlen(resp));

        ControlData_t ctrlData;
        memset(&ctrlData, 0, sizeof(ctrlData));
        ctrlData.cmdType = 0;
        for (int i = 0; i < 4; i++) ctrlData.motor_throttle[i] = _motors[i];
        ctrlData.armed = _armed;
        ctrlData.timestamp_ms = xTaskGetTickCount();
        _ctrlQueue.sendToBack(ctrlData, 0);

    } else if (strcmp(tok, "ARM") == 0) {
        _armed = true;
        _serial.send((uint8_t*)"OK: ARMED\r\n", 11);
        ControlData_t ctrlData;
        memset(&ctrlData, 0, sizeof(ctrlData));
        ctrlData.cmdType = 0;
        for (int i = 0; i < 4; i++) ctrlData.motor_throttle[i] = _motors[i];
        ctrlData.armed = true;
        ctrlData.timestamp_ms = xTaskGetTickCount();
        _ctrlQueue.sendToBack(ctrlData, 0);

    } else if (strcmp(tok, "DISARM") == 0 || strcmp(tok, "D") == 0 ||
               strcmp(tok, "STOP") == 0 || strcmp(tok, "ESTOP") == 0) {
        _armed = false;
        for (int i = 0; i < 4; i++) _motors[i] = 0;
        _serial.send((uint8_t*)"OK: STOPPED\r\n", 13);
        ControlData_t ctrlData;
        memset(&ctrlData, 0, sizeof(ctrlData));
        ctrlData.armed = false;
        ctrlData.timestamp_ms = xTaskGetTickCount();
        _ctrlQueue.sendToBack(ctrlData, 0);

    } else if (strcmp(tok, "STATUS") == 0 || strcmp(tok, "S") == 0) {
        snprintf(resp, sizeof(resp),
            "STATUS: armed=%d M0=%u M1=%u M2=%u M3=%u\r\n",
            _armed, _motors[0], _motors[1], _motors[2], _motors[3]);
        _serial.send((uint8_t*)resp, strlen(resp));

    } else if (strcmp(tok, "TEL") == 0) {
        char* sub = strtok(nullptr, " \r\n");
        if (sub) for (char* p = sub; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
        if (sub && strcmp(sub, "OFF") == 0) {
            _telEnabled = false;
            _serial.send((uint8_t*)"OK: TEL OFF\r\n", 13);
        } else {
            _telEnabled = true;
            _serial.send((uint8_t*)"OK: TEL ON\r\n", 12);
        }

    } else if (strcmp(tok, "PID") == 0) {
        // PID <R|P|Y> <P|I|D> <value>  or  PID RESET
        char* axs = strtok(nullptr, " \r\n");
        if (axs) for (char* p = axs; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

        if (axs && strcmp(axs, "RESET") == 0) {
            ControlData_t ctrlData;
            memset(&ctrlData, 0, sizeof(ctrlData));
            ctrlData.cmdType = 2;
            ctrlData.timestamp_ms = xTaskGetTickCount();
            _ctrlQueue.sendToBack(ctrlData, 0);
            _serial.send((uint8_t*)"OK: PID RESET\r\n", 14);
            return;
        }

        char* gn  = strtok(nullptr, " \r\n");
        char* vl  = strtok(nullptr, " \r\n");
        if (!axs || !gn || !vl) {
            _serial.send((uint8_t*)"ERR: PID <R|P|Y> <P|I|D> <value>\r\n", 36);
            return;
        }
        for (char* p = axs; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
        for (char* p = gn; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

        uint8_t axis = 255, gain = 255;
        if (strcmp(axs, "R") == 0 || strcmp(axs, "ROLL") == 0) axis = 0;
        else if (strcmp(axs, "P") == 0 || strcmp(axs, "PITCH") == 0) axis = 1;
        else if (strcmp(axs, "Y") == 0 || strcmp(axs, "YAW") == 0) axis = 2;
        else { _serial.send((uint8_t*)"ERR: axis must be R/P/Y\r\n", 25); return; }

        if (strcmp(gn, "P") == 0 || strcmp(gn, "KP") == 0) gain = 0;
        else if (strcmp(gn, "I") == 0 || strcmp(gn, "KI") == 0) gain = 1;
        else if (strcmp(gn, "D") == 0 || strcmp(gn, "KD") == 0) gain = 2;
        else { _serial.send((uint8_t*)"ERR: gain must be P/I/D\r\n", 25); return; }

        float val = (float)strtod(vl, nullptr);

        ControlData_t ctrlData;
        memset(&ctrlData, 0, sizeof(ctrlData));
        ctrlData.cmdType = 1;
        ctrlData.pid_axis = axis;
        ctrlData.pid_gain = gain;
        ctrlData.pid_value = val;
        ctrlData.timestamp_ms = xTaskGetTickCount();
        _ctrlQueue.sendToBack(ctrlData, 0);

        snprintf(resp, sizeof(resp), "OK: PID %c%c = %.4f\r\n",
            "RPY"[axis], "PID"[gain], val);
        _serial.send((uint8_t*)resp, strlen(resp));

    } else if (strcmp(tok, "HELP") == 0 || strcmp(tok, "?") == 0) {
        _serial.send((uint8_t*)
            "MOTOR <n|ALL> <0-1000>  set motor speed\r\n"
            "ARM/DISARM/STOP/ESTOP   arm/stop motors\r\n"
            "PID <R|P|Y> <P|I|D> <v> tune PID gains\r\n"
            "STATUS                  show status\r\n"
            "TEL ON/OFF              serial plot CSV\r\n",
            220);

    } else {
        snprintf(resp, sizeof(resp), "ERR: unknown cmd '%s', try HELP\r\n", tok);
        _serial.send((uint8_t*)resp, strlen(resp));
    }
}

void LoraTask::_sendTelemetry() {
    // CSV: roll_rate,pitch_rate,yaw_rate,m0,m1,m2,m3 (rate*10 deg/s, motor raw)
    int rx = (int)(RAD2DEG(_lastAtt.roll_rate)  * 10.0f);
    int ry = (int)(RAD2DEG(_lastAtt.pitch_rate) * 10.0f);
    int rz = (int)(RAD2DEG(_lastAtt.yaw_rate)   * 10.0f);

    char buf[64];
    int len = snprintf(buf, sizeof(buf),
        "%d,%d,%d,%d,%d,%d,%d\r\n",
        rx, ry, rz,
        ControlTask::motor_throttle[0], ControlTask::motor_throttle[1],
        ControlTask::motor_throttle[2], ControlTask::motor_throttle[3]);

    if (len > 0 && len < (int)sizeof(buf)) {
        _serial.send((uint8_t*)buf, len);
    }
}

void LoraTask::taskFunction() {
    static uint8_t buf[256];

    for (;;) {
        // 解锁时加速轮询以提速遥测
        TickType_t serialTO = _armed ? pdMS_TO_TICKS(5) : pdMS_TO_TICKS(100);
        auto message = _fromLoraSerialQueue.receive(serialTO);
        if (message) {
            uint16_t len = message->length;
            if (len > 255) len = 255;
            memcpy(buf, message->data, len);
            buf[len] = '\0';

            if (len > 0) {
                DBGQ.sendToBack(buf, 0);
                parseCommand((const char*)buf);
            }
        }

        // try to update latest attitude (non-blocking)
        auto att = _attQueue.receive(pdMS_TO_TICKS(0));
        if (att) {
            _lastAtt = *att;
            _hasAtt = true;
        }

        // 解锁时 ~200Hz CSV，未解锁静默
        if (_armed && _telEnabled && _hasAtt) {
            TickType_t now = xTaskGetTickCount();
            if ((now - _lastTelTick) >= pdMS_TO_TICKS(5)) {
                _lastTelTick = now;
                _sendTelemetry();
            }
        }
    }
}



LoraSerialTask::LoraSerialTask(UART_HandleTypeDef *huart,
                               FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                               FreeRTOS::Queue<DXLR01::LoraMessage_t> &toLoraTaskQueue,
                               const char* taskName,
                               UBaseType_t priority,
                               configSTACK_DEPTH_TYPE stackDepth) :
    SerialTaskBase(huart, rxQueue, taskName, priority, stackDepth),
    _toLoraTaskQueue(toLoraTaskQueue) {}

void LoraSerialTask::rxProcess(uint8_t *rxData, uint16_t len) {
    DXLR01::LoraMessage_t msg;
    msg.length = len;
    if (len > sizeof(msg.data)) len = sizeof(msg.data);
    memcpy(msg.data, rxData, len);
    msg.timestamp_ms = FreeRTOS::Kernel::getTickCount();

    _toLoraTaskQueue.sendToBack(msg);
}
