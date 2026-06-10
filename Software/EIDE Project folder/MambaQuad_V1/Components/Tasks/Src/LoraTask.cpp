#include "LoraTask.h"
#include "MagTask.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

LoraTask::LoraTask(LoraSerialTask &serial,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &fromLoraSerialQueue,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &loraQueue,
    FreeRTOS::Queue<ControlData_t> &ctrlQueue) :
    Task(tskIDLE_PRIORITY + 2, 512, "LoRa"),
    _serial(serial),
    _fromLoraSerialQueue(fromLoraSerialQueue),
    _loraQueue(loraQueue),
    _ctrlQueue(ctrlQueue) {}

bool LoraTask::init(uint8_t channel, uint8_t level, DXLR01::TransMode mode,
                    uint16_t address, uint8_t baud) {
    this->delay(pdMS_TO_TICKS(200));

    // void(channel);
    // void(level);
    // void(mode);
    // void(address);
    // void(baud);

    // _lora.begin(loraSendCb, loraReceiveCb, this);

    // // 信道 0x01, 速率等级 2, 透传模式, 地址 0x0001, 波特率 7=115200: 0x01, 2, DXLR01::TransMode::TRANSPARENT, 0x0001, 7
    // bool ok = _lora.configure(channel, level, mode, address, baud);
    
    return true;
}

void LoraTask::parseCommand(const char* cmd) {
    static char resp[256];
    char cmdCopy[256];
    strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
    cmdCopy[sizeof(cmdCopy) - 1] = '\0';

    // 拆第一个 token
    char* tok = strtok(cmdCopy, " \r\n");
    if (!tok) return;

    // 手动转大写
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

        // 发给 ControlTask
        ControlData_t ctrlData;
        for (int i = 0; i < 4; i++) ctrlData.motor_throttle[i] = _motors[i];
        ctrlData.armed = _armed;
        ctrlData.timestamp_ms = xTaskGetTickCount();
        _ctrlQueue.sendToBack(ctrlData, 0);

    } else if (strcmp(tok, "ARM") == 0) {
        _armed = true;
        _serial.send((uint8_t*)"OK: ARMED\r\n", 11);
        // 更新 ControlTask
        ControlData_t ctrlData;
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

    // } else if (strcmp(tok, "MAGCAL") == 0) {
    //     if (g_magCal.valid) {
    //         snprintf(resp, sizeof(resp),
    //                  "MAG: ox=%.6f oy=%.6f oz=%.6f\r\n"
    //                  "MAG: sx=%.6f sy=%.6f sz=%.6f\r\n",
    //                  g_magCal.offset_x, g_magCal.offset_y, g_magCal.offset_z,
    //                  g_magCal.scale_x, g_magCal.scale_y, g_magCal.scale_z);
    //     } else {
    //         snprintf(resp, sizeof(resp), "MAGCAL: not yet calibrated\r\n");
    //     }
    //     _serial.send((uint8_t*)resp, strlen(resp));

    } else if (strcmp(tok, "HELP") == 0 || strcmp(tok, "?") == 0) {
        _serial.send((uint8_t*)
            "MOTOR <n|ALL> <0-1000>  set motor speed\r\n"
            "ARM/DISARM/STOP/ESTOP   arm/stop motors\r\n"
            "STATUS                    show status\r\n",
            120);

    } else {
        snprintf(resp, sizeof(resp), "ERR: unknown cmd '%s', try HELP\r\n", tok);
        _serial.send((uint8_t*)resp, strlen(resp));
    }
}

void LoraTask::taskFunction() {
    static uint8_t buf[256];

    for (;;) {
        auto message = _fromLoraSerialQueue.receive(pdMS_TO_TICKS(500));
        if (message) {
            uint16_t len = message->length;
            if (len > 255) len = 255;
            memcpy(buf, message->data, len);
            buf[len] = '\0';

            if (len > 0) {
                // 回显到 USB 调试口
                DBGQ.sendToBack(buf, 0);
                parseCommand((const char*)buf);
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
