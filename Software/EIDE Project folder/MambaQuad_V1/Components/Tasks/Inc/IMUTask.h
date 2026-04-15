#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "icm42688p.h"
#include "data_types.h"

class IMUTask : public FreeRTOS::Task {
    public:
        IMUTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                uint16_t cs_pin, FreeRTOS::Queue<IMUData_t> &queue);

        bool init();
        static void intCallback_IRQ();

    private:
        void taskFunction() override;

        void _calibrateBias(uint16_t samples = 100);
        IMUData_t _biasing(IMUData_t &data);

        ICM42688P _icm42688p;
        static IMUTask *_instance;

        IMUData_t _imuData_Raw;
        IMUData_t _imuData_Bias;
        IMUData_t _imuData_Calib;

        FreeRTOS::Queue<IMUData_t> &_imuQueue;
};
