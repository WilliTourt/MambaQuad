#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "dshot.h"
#include "pid.h"

#include "data_types.h"

#define PID_ROLL_RATE_KP  160.0f
#define PID_ROLL_RATE_KI  120.0f
#define PID_ROLL_RATE_KD  2.2f

// #define PID_PITCH_RATE_KP 0.15f
// #define PID_PITCH_RATE_KI 0.01f
// #define PID_PITCH_RATE_KD 0.005f

// #define PID_YAW_RATE_KP   0.25f
// #define PID_YAW_RATE_KI   0.01f
// #define PID_YAW_RATE_KD   0.005f

#define PID_PITCH_RATE_KP 0.0f
#define PID_PITCH_RATE_KI 0.0f
#define PID_PITCH_RATE_KD 0.0f

#define PID_YAW_RATE_KP   0.0f
#define PID_YAW_RATE_KI   0.0f
#define PID_YAW_RATE_KD   0.0f

#define PID_OUT_LIMIT     400.0f



class ControlTask : public FreeRTOS::Task {
    public:
        ControlTask(DShot &m1, DShot &m2, DShot &m3, DShot &m4,
                    FreeRTOS::Queue<ControlData_t> *ctrlQueue,
                    FreeRTOS::Queue<AttitudeData_t> *attQueue);

        static volatile uint16_t motor_throttle[4];
        static void send();

        bool init();

    private:
        void taskFunction() override;

        void _arm();
        void _disarm();
        void _handlePIDCmd(uint8_t axis, uint8_t gain, float value);

        void _enableSending();
        void _disableSending();

        DShot _m1, _m2, _m3, _m4;
        PIDCtrller _pid_roll_rate, _pid_pitch_rate, _pid_yaw_rate;
        // PIDCtrller _pid_roll_angle, _pid_pitch_angle, _pid_yaw_angle;

        FreeRTOS::Queue<ControlData_t>   *_ctrlQueue;
        FreeRTOS::Queue<AttitudeData_t>  *_attQueue;

        AttitudeData_t _att;
        uint16_t _baseThrottle = 0;
        bool _armed = false;
        bool _hasIMU = false;
};
