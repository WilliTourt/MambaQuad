#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "icp10111.h"
#include "data_types.h"

class BaroTask : public FreeRTOS::Task {
    public:
        BaroTask(I2C_HandleTypeDef *hi2c,
                 ICP10111::ICP10111_MeasurementMode mode, FreeRTOS::Queue<BaroData_t> &queue);

        bool init();

    private:
        void taskFunction() override;

        ICP10111 _icp10111;
        BaroData_t _baroData;
        FreeRTOS::Queue<BaroData_t> &_baroQueue;

        class AlphaBetaFilter {
            public:
                AlphaBetaFilter(float alpha, float beta, float dt);

                float update(float measurement);

                inline float getX() const { return _x_hat; }
                inline float getDX() const { return _dx_hat; }

            private:
                void _init(float initial_x, float initial_dx = 0.0f);

                float _alpha;
                float _beta;
                float _dt;

                float _x_hat;
                float _x_predict;
                float _dx_hat;
                float _dx_predict;

                bool _initialized = false;
        };

        AlphaBetaFilter _altFilter;
        AlphaBetaFilter _pressureFilter;
};
