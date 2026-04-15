#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "qmc5883p.h"
#include "data_types.h"

class MagTask : public FreeRTOS::Task {
    public:
        MagTask(I2C_HandleTypeDef *hi2c,
                QMC5883P::QMC5883P_Mode mode, QMC5883P::QMC5883P_Spd spd,
                FreeRTOS::Queue<MagData_t> &queue);

        bool init();

    private:
        void taskFunction() override;
        QMC5883P _qmc5883p;

        MagData_t _magRawData;
        FreeRTOS::Queue<MagData_t> &_magQueue;
};
