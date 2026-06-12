#include "DBGTask.h"
#include "ControlTask.h"
#include <cstring>
#include <cstdio>
#include <cstdlib>

FreeRTOS::Queue<uint8_t*> DBGQ(32);

// Forward declare CDC functions from usbd_cdc_if.c
extern "C" {
    int usb_rx_available(void);
    int usb_rx_copy(char* dst, int maxlen);
}

#ifdef USB_AS_DEBUG
DBGTask::DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue,
                 FreeRTOS::Queue<uint8_t*> &generalDebugQueue,
                 FreeRTOS::Queue<ControlData_t> &ctrlQueue,
                 FreeRTOS::Queue<AttitudeData_t> &attQueue) :
                 Task(tskIDLE_PRIORITY + 1, 512, "DBG"),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue),
                 _gpsQueue(gpsQueue),
                 _generalDebugQueue(generalDebugQueue),
                 _ctrlQueue(ctrlQueue),
                 _attQueue(attQueue),
                 _debug(true, true, true) {}
#else
DBGTask::DBGTask(UART_HandleTypeDef *huart,
                 FreeRTOS::Queue<IMUData_t> &imuQueue,
                 FreeRTOS::Queue<MagData_t> &magQueue,
                 FreeRTOS::Queue<BaroData_t> &baroQueue,
                 FreeRTOS::Queue<GPSData_t> &gpsQueue,
                 FreeRTOS::Queue<uint8_t*> &generalDebugQueue) :
                 Task(tskIDLE_PRIORITY + 1, 512, "DBG"),
                 _imuQueue(imuQueue),
                 _magQueue(magQueue),
                 _baroQueue(baroQueue),
                 _gpsQueue(gpsQueue),
                 _generalDebugQueue(generalDebugQueue),
                 _debug(huart, true, true, true) {}
#endif

void DBGTask::_sendUsbResp(const char* msg) {
    _debug.log("%s", msg);
}

void DBGTask::_processUsbCmd(const char* cmd) {
    char cmdCopy[256];
    strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
    cmdCopy[sizeof(cmdCopy) - 1] = '\0';

    char* tok = strtok(cmdCopy, " \r\n");
    if (!tok) return;
    for (char* p = tok; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

    // ── PID commands ──
    if (strcmp(tok, "PID") == 0) {
        char* axs = strtok(nullptr, " \r\n");
        if (!axs) { _sendUsbResp("ERR: PID <R|P|Y|AR|AP|AY> <P|I|D> <v> | ON | OFF | RESET\r\n"); return; }
        for (char* p = axs; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

        if (strcmp(axs, "RESET") == 0) {
            ControlData_t ctrl;
            memset(&ctrl, 0, sizeof(ctrl));
            ctrl.cmdType = 2;
            _ctrlQueue.sendToBack(ctrl, 0);
            _sendUsbResp("OK: PID RESET\r\n");
            return;
        }
        if (strcmp(axs, "ON") == 0) {
            ControlData_t ctrl;
            memset(&ctrl, 0, sizeof(ctrl));
            ctrl.cmdType = 4; ctrl.pid_axis = 1;
            _ctrlQueue.sendToBack(ctrl, 0);
            _sendUsbResp("OK: PID ON\r\n");
            return;
        }
        if (strcmp(axs, "OFF") == 0) {
            ControlData_t ctrl;
            memset(&ctrl, 0, sizeof(ctrl));
            ctrl.cmdType = 4; ctrl.pid_axis = 0;
            _ctrlQueue.sendToBack(ctrl, 0);
            _sendUsbResp("OK: PID OFF\r\n");
            return;
        }

        char* gn = strtok(nullptr, " \r\n");
        char* vl = strtok(nullptr, " \r\n");
        if (!gn || !vl) { _sendUsbResp("ERR: PID <R|P|Y|AR|AP|AY> <P|I|D> <v>\r\n"); return; }
        for (char* p = gn; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

        uint8_t axis = 255, gain = 255;
        if (strcmp(axs, "R") == 0 || strcmp(axs, "ROLL") == 0) axis = 0;
        else if (strcmp(axs, "P") == 0 || strcmp(axs, "PITCH") == 0) axis = 1;
        else if (strcmp(axs, "Y") == 0 || strcmp(axs, "YAW") == 0) axis = 2;
        else if (strcmp(axs, "AR") == 0 || strcmp(axs, "RA") == 0) axis = 3;
        else if (strcmp(axs, "AP") == 0 || strcmp(axs, "PA") == 0) axis = 4;
        else if (strcmp(axs, "AY") == 0 || strcmp(axs, "YA") == 0) axis = 5;
        else { _sendUsbResp("ERR: axis R/P/Y/AR/AP/AY\r\n"); return; }

        if (strcmp(gn, "P") == 0 || strcmp(gn, "KP") == 0) gain = 0;
        else if (strcmp(gn, "I") == 0 || strcmp(gn, "KI") == 0) gain = 1;
        else if (strcmp(gn, "D") == 0 || strcmp(gn, "KD") == 0) gain = 2;
        else { _sendUsbResp("ERR: gain P/I/D\r\n"); return; }

        float val = (float)strtod(vl, nullptr);
        ControlData_t ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.cmdType = 1;
        ctrl.pid_axis = axis; ctrl.pid_gain = gain; ctrl.pid_value = val;
        _ctrlQueue.sendToBack(ctrl, 0);
        char resp[64];
        snprintf(resp, sizeof(resp), "OK: PID %c%c%c = %.4f\r\n",
            axis < 3 ? "RPY"[axis] : "RPY"[axis-3],
            axis < 3 ? ' ' : 'A', "PID"[gain], val);
        _sendUsbResp(resp);
        return;
    }

    // ── ARM/DISARM ──
    if (strcmp(tok, "ARM") == 0) {
        _armed = true;
        ControlData_t ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.armed = true;
        _ctrlQueue.sendToBack(ctrl, 0);
        _sendUsbResp("OK: ARMED\r\n");
        return;
    }
    if (strcmp(tok, "DISARM") == 0 || strcmp(tok, "D") == 0 ||
        strcmp(tok, "STOP") == 0 || strcmp(tok, "ESTOP") == 0) {
        _armed = false;
        for (int i = 0; i < 4; i++) _motors[i] = 0;
        ControlData_t ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.armed = false;
        _ctrlQueue.sendToBack(ctrl, 0);
        _sendUsbResp("OK: STOPPED\r\n");
        return;
    }

    // ── MOTOR ──
    if (strcmp(tok, "MOTOR") == 0 || strcmp(tok, "M") == 0) {
        char* ns = strtok(nullptr, " \r\n");
        char* vs = strtok(nullptr, " \r\n");
        if (!ns || !vs) { _sendUsbResp("ERR: MOTOR <n|ALL> <0-1000>\r\n"); return; }
        uint16_t sp = (uint16_t)strtoul(vs, nullptr, 10);
        if (sp > 1000) sp = 1000;
        ControlData_t ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.cmdType = 0;
        ctrl.armed = _armed;  // ← 保持当前ARM状态！
        if (strcmp(ns, "ALL") == 0) {
            for (int i = 0; i < 4; i++) { _motors[i] = sp; ctrl.motor_throttle[i] = sp; }
        } else {
            int n = strtoul(ns, nullptr, 10);
            if (n < 0 || n > 3) { _sendUsbResp("ERR: motor 0-3\r\n"); return; }
            _motors[n] = sp;
            for (int i = 0; i < 4; i++) ctrl.motor_throttle[i] = _motors[i];
        }
        _ctrlQueue.sendToBack(ctrl, 0);
        char resp[48];
        snprintf(resp, sizeof(resp), "OK: MOTOR %s=%u\r\n", ns, sp);
        _sendUsbResp(resp);
        return;
    }

    // ── AT (angle target) ──
    if (strcmp(tok, "AT") == 0) {
        char* axs = strtok(nullptr, " \r\n");
        char* vl = strtok(nullptr, " \r\n");
        if (!axs || !vl) { _sendUsbResp("ERR: AT <R|P> <deg>\r\n"); return; }
        for (char* p = axs; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
        uint8_t axis = 255;
        if (strcmp(axs, "R") == 0) axis = 0;
        else if (strcmp(axs, "P") == 0) axis = 1;
        else { _sendUsbResp("ERR: AT R/P\r\n"); return; }
        float deg = (float)strtod(vl, nullptr);
        ControlData_t ctrl;
        memset(&ctrl, 0, sizeof(ctrl));
        ctrl.cmdType = 3;
        ctrl.pid_axis = axis;
        ctrl.pid_value = deg * 0.0174533f;
        _ctrlQueue.sendToBack(ctrl, 0);
        char resp[48];
        snprintf(resp, sizeof(resp), "OK: AT %c=%.1f deg\r\n", "RP"[axis], deg);
        _sendUsbResp(resp);
        return;
    }

    // ── TEL (SerialPlot telemetry) ──
    if (strcmp(tok, "TEL") == 0) {
        char* sub = strtok(nullptr, " \r\n");
        if (sub) for (char* p = sub; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;
        if (sub && strcmp(sub, "OFF") == 0) {
            _telEnabled = false;
            _sendUsbResp("OK: TEL OFF\r\n");
        } else {
            _telEnabled = true;
            _sendUsbResp("OK: TEL ON\r\n");
        }
        return;
    }

    _sendUsbResp("ERR: unknown cmd\r\n");
}

void DBGTask::taskFunction() {
    for (;;) {
        #ifdef USB_AS_DEBUG
        {
            static char lineBuf[256];
            static int lineIdx = 0;
            if (usb_rx_available()) {
                char buf[64];
                int n = usb_rx_copy(buf, sizeof(buf));
                for (int i = 0; i < n; i++) {
                    if (buf[i] == '\r' || buf[i] == '\n') {
                        if (lineIdx > 0) {
                            lineBuf[lineIdx] = '\0';
                            _processUsbCmd(lineBuf);
                            lineIdx = 0;
                        }
                    } else if (lineIdx < 255) {
                        lineBuf[lineIdx++] = buf[i];
                    }
                }
            }
        }
        #endif

        // ── 遥测 CSV (SerialPlot) ──
        #ifdef USB_AS_DEBUG
        {
            auto att = _attQueue.receive(pdMS_TO_TICKS(0));
            if (att) { _lastAtt = *att; _hasAtt = true; }
            if (_telEnabled && _hasAtt) {
                TickType_t now = xTaskGetTickCount();
                if ((now - _lastTelTick) >= pdMS_TO_TICKS(5)) {
                    _lastTelTick = now;
                    char buf[64];
                    int len = snprintf(buf, sizeof(buf),
                        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                        (int)(_lastAtt.roll  * 57.29578f),
                        (int)(_lastAtt.pitch * 57.29578f),
                        (int)(_lastAtt.yaw   * 57.29578f),
                        (int)(_lastAtt.roll_rate  * 57.29578f),
                        (int)(_lastAtt.pitch_rate * 57.29578f),
                        (int)(_lastAtt.yaw_rate   * 57.29578f),
                        ControlTask::motor_throttle[0], ControlTask::motor_throttle[1],
                        ControlTask::motor_throttle[2], ControlTask::motor_throttle[3]);
                    if (len > 0 && len < (int)sizeof(buf)) {
                        CDC_Transmit_FS((uint8_t*)buf, len);
                    }
                }
            }
        }
        #endif

        #if (DBG_ENABLE_GENERAL == 1)
            if (auto generalData = _generalDebugQueue.receive(0)) {
                _debug.log("%s%sGeneral:%s %s @%lums\r\n",
                           BOLD, COLOR_DARK_BLUE, CLR,
                           *generalData, xTaskGetTickCount());
                continue;
            }
        #endif

        #if (DBG_ENABLE_GPS == 1)
            if (auto gpsData = _gpsQueue.receive(pdMS_TO_TICKS(50))) {
                _debug.log("%s%sGPS:%s Lat=%.6f Lon=%.6f Alt=%.2f SatInView=%d SatInUse=%d @%lums\r\n",
                           BOLD, COLOR_DARK_GREEN, CLR,
                           gpsData->lat, gpsData->lon, gpsData->elv, gpsData->satInView, gpsData->satInUse, gpsData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_IMU == 1)
            if (auto imuData = _imuQueue.receive(0)) {
                _debug.log("%s%sIMU:%s aX=%6.2f aY=%6.2f aZ=%6.2f gX=%6.2f gY=%6.2f gZ=%6.2f @%lums\r\n",
                           BOLD, COLOR_DARK_YELLOW, CLR,
                           imuData->ax, imuData->ay, imuData->az,
                           imuData->gx, imuData->gy, imuData->gz,
                           imuData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_MAG == 1)
            if (auto magData = _magQueue.receive(0)) {
                _debug.log("%s%sMag:%s X=%6.2fm Y=%6.2fm Z=%6.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_MAGENTA, CLR,
                           magData->mx, magData->my, magData->mz, magData->timestamp_ms);
                continue;
            }
        #endif

        #if (DBG_ENABLE_BARO == 1)
            if (auto baroData = _baroQueue.receive(0)) {
                _debug.log("%s%sBaro:%s P=%8.2fPa A=%7.2fm @%lums\r\n",
                           BOLD, COLOR_DARK_CYAN, CLR,
                           baroData->pressure_Pa, baroData->altitude_m, baroData->timestamp_ms);
                continue;
            }
        #endif

        this->delay(pdMS_TO_TICKS(7));
    }
}
