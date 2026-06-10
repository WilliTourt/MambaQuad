#include "ControlTask.h"
#include "DBGTask.h"

volatile uint16_t ControlTask::motor_throttle[4] = {0, 0, 0, 0};

static ControlTask *instance = nullptr;

ControlTask::ControlTask(DShot &m1, DShot &m2, DShot &m3, DShot &m4,
                         FreeRTOS::Queue<ControlData_t> *ctrlQueue,
                         FreeRTOS::Queue<AttitudeData_t> *attQueue) :
    Task(tskIDLE_PRIORITY + 4, 512, "CTRL"),
    _m1(m1), _m2(m2), _m3(m3), _m4(m4),
    _pid_roll_rate(PID_ROLL_RATE_KP, PID_ROLL_RATE_KI, PID_ROLL_RATE_KD),
    _pid_pitch_rate(PID_PITCH_RATE_KP, PID_PITCH_RATE_KI, PID_PITCH_RATE_KD),
    _pid_yaw_rate(PID_YAW_RATE_KP, PID_YAW_RATE_KI, PID_YAW_RATE_KD),
    // _pid_roll_angle,
    // _pid_pitch_angle,
    // _pid_yaw_angle,
    _ctrlQueue(ctrlQueue),
    _attQueue(attQueue) {
    instance = this;
    // _pid_pitch_rate.setDeadzone(0.35);  // 0.35 rad/s = 20.05 deg/s
    // _pid_roll_rate.setDeadzone(0.35);
    // _pid_yaw_rate.setDeadzone(0.35);
    _att = {};
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
    this->delay(pdMS_TO_TICKS(2000));

    _pid_roll_rate.reset();
    _pid_pitch_rate.reset();
    _pid_yaw_rate.reset();
    _pid_roll_rate.setTarget(0.0f);
    _pid_pitch_rate.setTarget(0.0f);
    _pid_yaw_rate.setTarget(0.0f);

    // 控制循环 ~500Hz → dt=0.002s，必须设对否则 I 项积分速度完全不对
    _pid_roll_rate.setSampleTime(0.002f);
    _pid_pitch_rate.setSampleTime(0.002f);
    _pid_yaw_rate.setSampleTime(0.002f);

    // 条件积分：只在 rate<1 rad/s 时积 I，手动掰它时 I 不累
    _pid_roll_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 1.0f);
    _pid_pitch_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 1.0f);
    _pid_yaw_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 1.0f);

    _armed = true;
    DBGQ.sendToBack((uint8_t*)"ControlTask: Motors armed.", 0);
}

void ControlTask::_disarm() {
    motor_throttle[0] = 0;
    motor_throttle[1] = 0;
    motor_throttle[2] = 0;
    motor_throttle[3] = 0;
    this->delay(pdMS_TO_TICKS(3000));
    _disableSending();

    _armed = false;
    DBGQ.sendToBack((uint8_t*)"ControlTask: Motors disarmed.", 0);
}

void ControlTask::_enableSending() {
    HAL_TIM_Base_Start_IT(&htim2);
}

void ControlTask::_disableSending() {
    HAL_TIM_Base_Stop_IT(&htim2);
}

void ControlTask::_handlePIDCmd(uint8_t axis, uint8_t gain, float value) {
    
    PIDCtrller* pid = nullptr;
    const char* axisName = nullptr;

    switch (axis) {
        case 0: pid = &_pid_roll_rate;  axisName = "ROLL"; break;
        case 1: pid = &_pid_pitch_rate; axisName = "PITCH"; break;
        case 2: pid = &_pid_yaw_rate;   axisName = "YAW"; break;
        default: return;
    }

    float kp = pid->getKp(), ki = pid->getKi(), kd = pid->getKd();
    const char* gainName = nullptr;
    switch (gain) {
        case 0: kp = value; gainName = "KP"; break;
        case 1: ki = value; gainName = "KI"; break;
        case 2: kd = value; gainName = "KD"; break;
        default: return;
    }
    pid->setParams(kp, ki, kd);

    char buf[64];
    snprintf(buf, sizeof(buf),
        "PID %s %s=%.3f (KP=%.3f KI=%.3f KD=%.3f)\r\n",
        axisName, gainName, value, kp, ki, kd);
    DBGQ.sendToBack((uint8_t*)buf, 0);
}

void ControlTask::taskFunction() {
    DBGQ.sendToBack((uint8_t*)"ControlTask started.", 0);
    for (;;) {
        // read latest attitude (non-blocking)
        auto att = _attQueue->receive(pdMS_TO_TICKS(1));
        if (att) {
            _att = *att;
            _hasIMU = true;
        }

        // drain all pending control commands (non-blocking)
        while (true) {
            auto cmd = _ctrlQueue->receive(pdMS_TO_TICKS(0));
            if (!cmd) break;

            if (cmd->cmdType == 1) {
                _handlePIDCmd(cmd->pid_axis, cmd->pid_gain, cmd->pid_value);
                continue;
            }

            if (cmd->cmdType == 2) {
                _pid_roll_rate.reset();
                _pid_pitch_rate.reset();
                _pid_yaw_rate.reset();
                DBGQ.sendToBack((uint8_t*)"PID RESET\r\n", 10);
                continue;
            }

            if (!_armed && cmd->armed) {
                DBGQ.sendToBack((uint8_t*)"ControlTask: Received arm command.", 0);
                _arm();
                continue;
            }

            if (_armed && !cmd->armed) {
                DBGQ.sendToBack((uint8_t*)"ControlTask: Received disarm command.", 0);
                _disarm();
                continue;
            }

            if (_armed) {
                _baseThrottle = cmd->motor_throttle[0];
                motor_throttle[0] = _baseThrottle;
                motor_throttle[1] = _baseThrottle;
                motor_throttle[2] = _baseThrottle;
                motor_throttle[3] = _baseThrottle;
            }
        }

        // ── PID 控制（解锁 + 有姿态数据 + 油门 > 怠速）──
        if (_armed && _hasIMU && _baseThrottle > 48) {
            float roll_out  = _pid_roll_rate.calc(_att.roll_rate,  PID_OUT_LIMIT, -PID_OUT_LIMIT);
            float pitch_out = _pid_pitch_rate.calc(_att.pitch_rate, PID_OUT_LIMIT, -PID_OUT_LIMIT);
            float yaw_out   = _pid_yaw_rate.calc(_att.yaw_rate,    PID_OUT_LIMIT, -PID_OUT_LIMIT);
            
            /*
                M4 M2
                  X    ↑
                M3 M1
            */

            // X-quad 混控
            int16_t t  = (int16_t)_baseThrottle;
            int16_t m1 = t + (int16_t)(-roll_out - pitch_out + yaw_out); // M0 右后
            int16_t m2 = t + (int16_t)(-roll_out + pitch_out + yaw_out); // M1 右前
            int16_t m3 = t + (int16_t)( roll_out - pitch_out - yaw_out); // M2 左后
            int16_t m4 = t + (int16_t)( roll_out + pitch_out - yaw_out); // M3 左前

            auto clamp = [](int16_t v) -> uint16_t {
                if (v < 48)  return 48;
                if (v > 1000) return 1000;
                return (uint16_t)v;
            };

            motor_throttle[0] = clamp(m1);
            motor_throttle[1] = clamp(m2);
            motor_throttle[2] = clamp(m3);
            motor_throttle[3] = clamp(m4);
        }

        this->delay(pdMS_TO_TICKS(2));
    }
}
