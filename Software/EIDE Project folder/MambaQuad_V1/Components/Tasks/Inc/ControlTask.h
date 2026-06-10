#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "dshot.h"
#include "pid.h"

#include "data_types.h"



class ControlTask : public FreeRTOS::Task {
    public:
        ControlTask(DShot &m1, DShot &m2, DShot &m3, DShot &m4,
                    FreeRTOS::Queue<ControlData_t> *queue);

        static volatile uint16_t motor_throttle[4];
        static void send();

        bool init();

    private:
        void taskFunction();

        void _arm();
        void _disarm();

        void _enableSending();
        void _disableSending();

        DShot _m1, _m2, _m3, _m4;
        PIDCtrller _pid;
        
        FreeRTOS::Queue<ControlData_t> *_queue;
        bool _armed = false;
};
