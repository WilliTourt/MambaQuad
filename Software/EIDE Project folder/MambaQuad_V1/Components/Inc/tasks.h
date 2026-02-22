#pragma once

#include <cstring>

#include "main.h"
#include "usart.h"
#include "usbd_cdc_if.h"
#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "icm42688p.h"
#include "icp10111.h"
#include "qmc5883p.h"
#include "data_types.h"

extern "C" {
    void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                         StackType_t **ppxIdleTaskStackBuffer,
                                         uint32_t *pulIdleTaskStackSize );

    void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                          StackType_t **ppxTimerTaskStackBuffer,
                                          uint32_t *pulTimerTaskStackSize );
}

#define USB_AS_DEBUG 1

#define DBG_ENABLE_IMU 0
#define DBG_ENABLE_MAG 0
#define DBG_ENABLE_BARO 1

class DBGTask : public FreeRTOS::Task {
    public:
    #if (USB_AS_DEBUG == 1)
        DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                FreeRTOS::Queue<MagData_t> &magQueue,
                FreeRTOS::Queue<BaroData_t> &baroQueue) :
                Task(tskIDLE_PRIORITY + 1, 256, "DBG"),
                _imuQueue(imuQueue),
                _magQueue(magQueue),
                _baroQueue(baroQueue) {}
    #else
        DBGTask(UART_HandleTypeDef *huart, FreeRTOS::Queue<MagData_t> &magQueue, FreeRTOS::Queue<BaroData_t> &baroQueue) :
            Task(tskIDLE_PRIORITY + 1, 256, "DBG"),
            _huart(huart),
            _magQueue(magQueue),
            _baroQueue(baroQueue) {}
    #endif

    private:
        void taskFunction() override {
            for (;;) {
                #if (DBG_ENABLE_IMU == 1)
                    auto imuData = _imuQueue.receive(portMAX_DELAY);
                    if (imuData) {
                        sprintf((char*)_dbg_buffer, "IMU: aX=%.2f aY=%.2f aZ=%.2f gX=%.2f gY=%.2f gZ=%.2f @%lums\r\n",
                                imuData->ax, imuData->ay, imuData->az,
                                imuData->gx, imuData->gy, imuData->gz,
                                imuData->timestamp_ms);

                        #if (USB_AS_DEBUG == 1)
                            CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                        #else
                            HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                        #endif
                    }
                #endif

                #if (DBG_ENABLE_MAG == 1)
                    auto magData = _magQueue.receive(portMAX_DELAY);
                    if (magData) {
                        sprintf((char*)_dbg_buffer, "Mag: X=%.2fm Y=%.2fm Z=%.2fm @%lums\r\n",
                                magData->mx, magData->my, magData->mz, magData->timestamp_ms);
                        
                        #if (USB_AS_DEBUG == 1)
                            CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                        #else
                            HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                        #endif
                    }
                #endif

                #if (DBG_ENABLE_BARO == 1)
                    auto baroData = _baroQueue.receive(portMAX_DELAY);
                    if (baroData) {
                        sprintf((char*)_dbg_buffer, "Baro: P=%.2fPa A=%.2fm @%lums\r\n",
                                baroData->pressure_Pa, baroData->altitude_m, baroData->timestamp_ms);
                        
                        #if (USB_AS_DEBUG == 1)
                            CDC_Transmit_FS(_dbg_buffer, strlen((char*)_dbg_buffer));
                        #else
                            HAL_UART_Transmit(_huart, _dbg_buffer, strlen((char*)_dbg_buffer), HAL_MAX_DELAY);
                        #endif
                    }
                #endif
            }
        }

        #if (USB_AS_DEBUG == 0)
            UART_HandleTypeDef *_huart;
        #endif

        FreeRTOS::Queue<IMUData_t> &_imuQueue;
        FreeRTOS::Queue<MagData_t> &_magQueue;
        FreeRTOS::Queue<BaroData_t> &_baroQueue;
        uint8_t _dbg_buffer[256];
};

class BlinkTask : public FreeRTOS::Task {
    public:
        // 构造函数：传递给基类优先级、栈大小、任务名
        BlinkTask() : Task(tskIDLE_PRIORITY + 2, 32, "Blink") {}

    private:
        void taskFunction() override {
            for (;;) {
                HAL_GPIO_TogglePin(LED_LF_GPIO_Port, LED_LF_Pin);
                HAL_GPIO_TogglePin(LED_RF_GPIO_Port, LED_RF_Pin);
                HAL_GPIO_TogglePin(LED_LR_GPIO_Port, LED_LR_Pin);
                HAL_GPIO_TogglePin(LED_RR_GPIO_Port, LED_RR_Pin);
                this->delayUntil(pdMS_TO_TICKS(500));
            }
        }
};





class IMUTask : public FreeRTOS::Task {
    public:
        IMUTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, FreeRTOS::Queue<IMUData_t> &queue) :
            Task(tskIDLE_PRIORITY + 3, 512, "IMU"),
            _icm42688p(hspi, cs_port, cs_pin),
            _imuQueue(queue) {}

        bool init();
        static void intCallback_IRQ();

    private:
        void taskFunction() override;
        // static void _drdyCallback(void *context);

        void _calibrateBias(uint16_t samples = 100);
        IMUData_t _biasing(IMUData_t &data);

        ICM42688P _icm42688p;
        static IMUTask *_instance;

        IMUData_t _imuData_Raw;
        IMUData_t _imuData_Bias;
        IMUData_t _imuData_Calib;

        // bool _bias_collected = false;

        FreeRTOS::Queue<IMUData_t> &_imuQueue;
};

class MagTask : public FreeRTOS::Task {
    public:
        MagTask(I2C_HandleTypeDef *hi2c, QMC5883P::QMC5883P_Mode mode, QMC5883P::QMC5883P_Spd spd, FreeRTOS::Queue<MagData_t> &queue) :
            Task(tskIDLE_PRIORITY + 3, 128, "Mag"),
            _qmc5883p(hi2c, mode, spd),
            _magQueue(queue) {}

        bool init();

    private:
        void taskFunction() override;
        QMC5883P _qmc5883p;

        MagData_t _magRawData;
        FreeRTOS::Queue<MagData_t> &_magQueue;
};

class BaroTask : public FreeRTOS::Task {
    public:
        BaroTask(I2C_HandleTypeDef *hi2c, FreeRTOS::Queue<BaroData_t> &queue) :
            Task(tskIDLE_PRIORITY + 3, 128, "Baro"),
            _icp10111(hi2c),
            _baroQueue(queue),
            _altFilter(0.1f, 0.006f, 0.03f),
            _pressureFilter(0.5f, 0.05f, 0.03f) {}

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
