#pragma once

#include <cstring>

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "ElegantDebug.h"

#include "icm42688p.h"
#include "icp10111.h"
#include "qmc5883p.h"
#include "ATGM336H.h"
#include "data_types.h"

extern "C" {
    void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                         StackType_t **ppxIdleTaskStackBuffer,
                                         uint32_t *pulIdleTaskStackSize );

    void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                          StackType_t **ppxTimerTaskStackBuffer,
                                          uint32_t *pulTimerTaskStackSize );
}

#ifdef USB_AS_DEBUG_PORT
    #define USB_AS_DEBUG
#endif

#define DBG_ENABLE_IMU 0
#define DBG_ENABLE_MAG 0
#define DBG_ENABLE_BARO 1
#define DBG_ENABLE_GPS 1

class DBGTask : public FreeRTOS::Task {
    public:
    #ifdef USB_AS_DEBUG
        DBGTask(FreeRTOS::Queue<IMUData_t> &imuQueue,
                FreeRTOS::Queue<MagData_t> &magQueue,
                FreeRTOS::Queue<BaroData_t> &baroQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue);
    #else
        DBGTask(UART_HandleTypeDef *huart,
                FreeRTOS::Queue<IMUData_t> &imuQueue,
                FreeRTOS::Queue<MagData_t> &magQueue,
                FreeRTOS::Queue<BaroData_t> &baroQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue);
    #endif

    private:
        void taskFunction() override;

        FreeRTOS::Queue<IMUData_t> &_imuQueue;
        FreeRTOS::Queue<MagData_t> &_magQueue;
        FreeRTOS::Queue<BaroData_t> &_baroQueue;
        FreeRTOS::Queue<GPSData_t> &_gpsQueue;
        
        ElegantDebug _debug;
};

// BASE CLASS OF SERIAL TASKS
class SerialTaskBase : public FreeRTOS::Task {
    public:

        struct RxPacket {
            uint16_t length;
            uint8_t data[256];
        };

        SerialTaskBase(UART_HandleTypeDef *huart,
                       FreeRTOS::Queue<RxPacket> &rxQueue,
                       const char* taskName,
                       UBaseType_t priority = tskIDLE_PRIORITY + 2,
                       configSTACK_DEPTH_TYPE stackDepth = 512);

        bool init();

        bool send(uint8_t* data, uint16_t len);
        // static void irqHandler(UART_HandleTypeDef *huart);
        static void rxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

    protected:
        void taskFunction() override final;
        virtual void rxProcess(uint8_t *data, uint16_t len) = 0;

        UART_HandleTypeDef* _huart;
        FreeRTOS::Queue<RxPacket> &_rxQueue;

    private:
        // uint8_t _rxByte;

        static constexpr uint16_t RX_BUF_SIZE = 256;
        uint8_t _rxBuffer[RX_BUF_SIZE];

        static SerialTaskBase* _instances[5];
};



class BlinkTask : public FreeRTOS::Task {
    public:
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
        IMUTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                uint16_t cs_pin, FreeRTOS::Queue<IMUData_t> &queue);

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



class LoraSerialTask : public SerialTaskBase {
    public:
        LoraSerialTask(UART_HandleTypeDef *huart,
                       FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                       FreeRTOS::Queue<LoraMessage_t> &toLoraTaskQueue,
                       const char* taskName = "LoraSerialTask",
                       UBaseType_t priority = tskIDLE_PRIORITY + 2,
                       configSTACK_DEPTH_TYPE stackDepth = 512);
        
    private:
        void rxProcess(uint8_t *data, uint16_t len) override;

        FreeRTOS::Queue<LoraMessage_t> &_toLoraTaskQueue;
};

class LoraTask : public FreeRTOS::Task {
    public:

        enum class LoRaMode {
            TRANSMIT,
            AT
        };

        enum class ATcmd {
            AT,
            RESET,
            DEFAULT,
            BAUD,
            PARI,
            HELP,
            LEVEL,
            MODE,
            SLEEP,
            SWITCH,
            CHANNEL,
            MAC,
            OPENKEY,
            KEY,
            PACKET,
            DRSSI,
            POWE,
            LBT,
            LRSSI,
            ERSSI
        };

        LoraTask(FreeRTOS::Queue<LoraMessage_t> &rxQueue,
                 LoraSerialTask &serial);

        bool init();

        void setMode(LoRaMode mode);
        bool sendAT(ATcmd cmd, uint8_t *response = nullptr);
        void sendData(const uint8_t *data, size_t len);

    private:
        void taskFunction() override;
        const char* _getATstr(ATcmd cmd);

        FreeRTOS::Queue<LoraMessage_t> &_fromLoraSerialQueue;
        LoraSerialTask &_serial;

        LoRaMode _mode;
};

class GPSSerialTask : public SerialTaskBase {
    public:
        GPSSerialTask(UART_HandleTypeDef *huart,
                      FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                      FreeRTOS::Queue<GPSData_t> &toGPSTaskQueue,
                      const char* taskName = "GPSSerialTask",
                      UBaseType_t priority = tskIDLE_PRIORITY + 2,
                      configSTACK_DEPTH_TYPE stackDepth = 1024);

    private:
        void rxProcess(uint8_t *data, uint16_t len) override;

        FreeRTOS::Queue<GPSData_t> &_toGPSTaskQueue;
        ATGM336H _gps;
};

class GPSTask : public FreeRTOS::Task {
    public:
        GPSTask(GPSSerialTask &serial,
                FreeRTOS::Queue<GPSData_t> &fromGPSSerialQueue,
                FreeRTOS::Queue<GPSData_t> &gpsQueue);

    private:
        void taskFunction() override;

        GPSSerialTask &_serial;
        FreeRTOS::Queue<GPSData_t> &_fromGPSSerialQueue;
        FreeRTOS::Queue<GPSData_t> &_gpsQueue;
};
