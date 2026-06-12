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
    _pid_roll_angle(PID_ROLL_ANGLE_KP, PID_ROLL_ANGLE_KI, PID_ROLL_ANGLE_KD),
    _pid_pitch_angle(PID_PITCH_ANGLE_KP, PID_PITCH_ANGLE_KI, PID_PITCH_ANGLE_KD),
    _pid_yaw_angle(PID_YAW_ANGLE_KP, PID_YAW_ANGLE_KI, PID_YAW_ANGLE_KD),
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

    {
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

        _pid_roll_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 0.15f);
        _pid_pitch_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 0.15f);
        _pid_yaw_rate.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 0.15f);
        _pid_roll_rate.setDerivativeMode(PIDCtrller::DerivativeMode_t::OnMeasurement);
        _pid_pitch_rate.setDerivativeMode(PIDCtrller::DerivativeMode_t::OnMeasurement);
        _pid_yaw_rate.setDerivativeMode(PIDCtrller::DerivativeMode_t::OnMeasurement);
    }

    {
        _pid_roll_angle.reset();
        _pid_pitch_angle.reset();
        _pid_yaw_angle.reset();
        _pid_roll_angle.setTarget(0.0f);
        _pid_pitch_angle.setTarget(0.0f);
        // _pid_yaw_angle.setTarget(0.0f);

        _pid_roll_angle.setSampleTime(0.002f);
        _pid_pitch_angle.setSampleTime(0.002f);
        _pid_yaw_angle.setSampleTime(0.002f);

        _pid_roll_angle.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 0.15f);
        _pid_pitch_angle.setIntegralMode(PIDCtrller::IntegralMode_t::Conditional, 0.15f);
    }

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
        case 0: pid = &_pid_roll_rate;  axisName = "ROLL_R"; break;
        case 1: pid = &_pid_pitch_rate; axisName = "PITCH_R"; break;
        case 2: pid = &_pid_yaw_rate;   axisName = "YAW_R"; break;
        case 3: pid = &_pid_roll_angle; axisName = "ROLL_A"; break;
        case 4: pid = &_pid_pitch_angle;axisName = "PITCH_A";break;
        case 5: pid = &_pid_yaw_angle;  axisName = "YAW_A"; break;
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
                _pid_roll_angle.reset();
                _pid_pitch_angle.reset();
                _pid_yaw_angle.reset();
                DBGQ.sendToBack((uint8_t*)"PID RESET\r\n", 10);
                continue;
            }

            if (cmd->cmdType == 3) {
                // AT: set angle target (value in rad, already converted in LoraTask)
                uint8_t ax = cmd->pid_axis;
                if (ax < 3) {
                    _angleTarget[ax] = cmd->pid_value;
                    if (ax == 0) _pid_roll_angle.setTarget(cmd->pid_value);
                    if (ax == 1) _pid_pitch_angle.setTarget(cmd->pid_value);
                    if (ax == 2) _pid_yaw_angle.setTarget(cmd->pid_value);
                    char buf[48];
                    snprintf(buf, sizeof(buf), "AT %c=%.2f deg\r\n",
                        "RPY"[ax], cmd->pid_value * 57.29578f);
                    DBGQ.sendToBack((uint8_t*)buf, 0);
                }
                continue;
            }

            if (cmd->cmdType == 4) {
                // PID ON/OFF: pid_axis=0→OFF, 1→ON
                _pidActive = (cmd->pid_axis == 1);
                char buf[32];
                snprintf(buf, sizeof(buf), "PID %s\r\n", _pidActive ? "ON" : "OFF");
                DBGQ.sendToBack((uint8_t*)buf, 0);
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

        /*
        Thinking:
        外圈角度环，输出的东西给到角速度环的输入。角度给出需要的角速度，从而改变角度
        内圈角速度环输出的直接是电机差速，电机差速越大扭矩越大，得到角加速度，然后使得角速度得到控制
        最终采样角度传回角度环
        */
        // PID Ctrl
        if (_armed && _pidActive && _hasIMU && _baseThrottle > 48) {
            // ── 20Hz 四阶巴特沃斯低通 (两节级联, fs=500Hz) ──
            // 65Hz衰减 -41dB, 延迟 ~21ms, 四阶陡降一刀切死
            float r_out, p_out;
            {
                // Section 1: fc=20Hz Q=0.541
                const float b10=0.0133f, b11=0.0266f, b12=0.0133f;
                const float a11=-1.6000f, a12=0.6532f;
                // Section 2: fc=20Hz Q=1.307
                const float b20=0.0133f, b21=0.0266f, b22=0.0133f;
                const float a21=-1.3762f, a22=0.4294f;

                static float r_x1=0, r_x2=0, r_y1=0, r_y2=0;
                static float r2_x1=0, r2_x2=0, r2_y1=0, r2_y2=0;
                static float p_x1=0, p_x2=0, p_y1=0, p_y2=0;
                static float p2_x1=0, p2_x2=0, p2_y1=0, p2_y2=0;

                // Roll: Section1 → Section2
                float rt = b10*_att.roll + b11*r_x1 + b12*r_x2 - a11*r_y1 - a12*r_y2;
                r_x2=r_x1; r_x1=_att.roll; r_y2=r_y1; r_y1=rt;
                r_out=b20*rt + b21*r2_x1 + b22*r2_x2 - a21*r2_y1 - a22*r2_y2;
                r2_x2=r2_x1; r2_x1=rt; r2_y2=r2_y1; r2_y1=r_out;

                // Pitch: Section1 → Section2
                float pt = b10*_att.pitch + b11*p_x1 + b12*p_x2 - a11*p_y1 - a12*p_y2;
                p_x2=p_x1; p_x1=_att.pitch; p_y2=p_y1; p_y1=pt;
                p_out=b20*pt + b21*p2_x1 + b22*p2_x2 - a21*p2_y1 - a22*p2_y2;
                p2_x2=p2_x1; p2_x1=pt; p2_y2=p2_y1; p2_y1=p_out;

                _pid_roll_rate.setTarget(
                    _pid_roll_angle.calc(r_out, PID_ANGLE_OUT_LIMIT, -PID_ANGLE_OUT_LIMIT));
                _pid_pitch_rate.setTarget(
                    _pid_pitch_angle.calc(p_out, PID_ANGLE_OUT_LIMIT, -PID_ANGLE_OUT_LIMIT));
            }

            // 速率环直接用原始数据（不加滤波，延迟代价太大）
            float roll_out  = _pid_roll_rate.calc(_att.roll_rate,  PID_RATE_OUT_LIMIT, -PID_RATE_OUT_LIMIT);
            float pitch_out = _pid_pitch_rate.calc(_att.pitch_rate, PID_RATE_OUT_LIMIT, -PID_RATE_OUT_LIMIT);
            float yaw_out   = _pid_yaw_rate.calc(_att.yaw_rate,    PID_RATE_OUT_LIMIT, -PID_RATE_OUT_LIMIT);
            
            /*
                 M4(CW) M2(CCW)
                       X        ↑
                M3(CCW) M1(CW)
            */

            // X-quad Mixer https://g413164351.github.io/pages/px4_mixer_tutorial.html
            int16_t t  = (int16_t)_baseThrottle;
            int16_t m1 = t + (int16_t)(-roll_out - pitch_out - yaw_out); // M1 RR (Right Rear)
            int16_t m2 = t + (int16_t)(-roll_out + pitch_out + yaw_out); // M2 RF (Right Front)
            int16_t m3 = t + (int16_t)( roll_out - pitch_out + yaw_out); // M3 LR
            int16_t m4 = t + (int16_t)( roll_out + pitch_out - yaw_out); // M4 LF

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

        this->delay(pdMS_TO_TICKS(2)); // ~500ms cycle
    }
}
