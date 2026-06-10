#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "qmc5883p.h"
#include "data_types.h"

// // 蓝牙 MAGCAL 命令可读取的全局校准结果
// struct MagCalResult {
//     float offset_x, offset_y, offset_z;
//     float scale_x, scale_y, scale_z;
//     bool valid;
// };
// extern MagCalResult g_magCal;

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
