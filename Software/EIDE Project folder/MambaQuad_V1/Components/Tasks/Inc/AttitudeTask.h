#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "data_types.h"

class AttitudeTask : public FreeRTOS::Task {
    public:
        AttitudeTask(FreeRTOS::Queue<IMUData_t>  &imuQueue,
                     FreeRTOS::Queue<MagData_t>  &magQueue,
                     FreeRTOS::Queue<AttitudeData_t> &attQueue);

    private:
        void taskFunction() override;

        void _update(const IMUData_t &imu, const MagData_t *mag, float dt);

        FreeRTOS::Queue<IMUData_t>  &_imuQueue;
        FreeRTOS::Queue<MagData_t>  &_magQueue;
        FreeRTOS::Queue<AttitudeData_t> &_attQueue;

        AttitudeData_t _att;
        uint32_t _lastIMUTime;
};
