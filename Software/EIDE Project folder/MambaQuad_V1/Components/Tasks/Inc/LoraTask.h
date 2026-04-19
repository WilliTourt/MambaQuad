#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "usart.h"
#include "SerialTaskBase.h"
#include "data_types.h"

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
        
        enum class LoRaTransMode {
            TRANSPARENT,
            DIRECTIONAL,
            BROADCAST
        };

        enum class ATcmd {
            AT, RESET, DEFAULT, BAUD, PARI, LEVEL, MODE,
            SLEEP, SWITCH, CHANNEL, MAC, OPENKEY, KEY, PACKET,
            DRSSI, POWE, LBT, LRSSI
        };

        LoraTask(FreeRTOS::Queue<LoraMessage_t> &rxQueue,
                 LoraSerialTask &serial);

        bool init();
        
        /**
         * @brief 配置 LoRa 模块参数
         * @param channel 信道号 (0x00-0x63)
         * @param level 速率等级 0-7 (0最慢最远, 7最快最近)
         * @param mode 传输模式
         * @param address 本机地址 (定点模式时需要)
         * @param baud 波特率代码 (3=9600, 7=115200, 0=不修改)
         * @return true 配置成功
         */
        bool configure(uint8_t channel = 0x00, 
                       uint8_t level = 2, 
                       LoRaTransMode mode = LoRaTransMode::TRANSPARENT,
                       uint16_t address = 0x0001,
                       uint8_t baud = 0);

        bool setMode(LoRaMode mode);
        bool sendAT(ATcmd cmd, const char* arg = nullptr);
        
        /**
         * @brief 发送数据
         * @param data 数据指针
         * @param len 数据长度
         * @note 根据当前传输模式自动添加前缀
         */
        void sendData(const uint8_t *data, size_t len);
        
        LoRaTransMode getTransMode() const { return _transMode; }
        uint16_t getAddress() const { return _address; }
        uint8_t getChannel() const { return _channel; }

    private:
        void taskFunction() override;
        const char* _getATstr(ATcmd cmd);

        FreeRTOS::Queue<LoraMessage_t> &_fromLoraSerialQueue;
        LoraSerialTask &_serial;

        LoRaMode _mode;
        LoRaTransMode _transMode;
        
        uint16_t _address;   // 本机地址 / 目标地址
        uint8_t _channel;    // 当前信道
};
