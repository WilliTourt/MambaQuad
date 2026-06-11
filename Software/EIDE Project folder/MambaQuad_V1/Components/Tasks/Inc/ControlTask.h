#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "dshot.h"
#include "pid.h"

#include "data_types.h"

#define PID_ROLL_RATE_KP  160.0f
#define PID_ROLL_RATE_KI  20.0f
#define PID_ROLL_RATE_KD  1.9f

#define PID_PITCH_RATE_KP 160.0f
#define PID_PITCH_RATE_KI 20.0f
#define PID_PITCH_RATE_KD 1.9f

#define PID_YAW_RATE_KP   80.0f
#define PID_YAW_RATE_KI   0.0f
#define PID_YAW_RATE_KD   0.2f

#define PID_ROLL_ANGLE_KP   12.0f
#define PID_ROLL_ANGLE_KI   5.0f
#define PID_ROLL_ANGLE_KD   0.0f

#define PID_PITCH_ANGLE_KP  12.0f
#define PID_PITCH_ANGLE_KI  5.0f
#define PID_PITCH_ANGLE_KD  0.0f

#define PID_YAW_ANGLE_KP   0.0f
#define PID_YAW_ANGLE_KI   0.0f
#define PID_YAW_ANGLE_KD   0.0f

#define PID_RATE_OUT_LIMIT  400.0f // Rate loop motor diff limit
#define PID_ANGLE_OUT_LIMIT 15.0f  // Angle loop speed limit in rad/s



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
        PIDCtrller _pid_roll_angle, _pid_pitch_angle, _pid_yaw_angle;

        FreeRTOS::Queue<ControlData_t>   *_ctrlQueue;
        FreeRTOS::Queue<AttitudeData_t>  *_attQueue;

        AttitudeData_t _att;
        float _angleTarget[3] = {0, 0, 0}; // [roll, pitch, yaw] in rad
        uint16_t _baseThrottle = 0;
        bool _armed = false;
        bool _pidActive = false;
        bool _hasIMU = false;
};
