#pragma once


#define LORA_USE_FREERTOS true // Set to 1 to enable FreeRTOS delays, 0 for HAL_Delay (blocking)

#include "main.h"

#if LORA_USE_FREERTOS == 1
    #include "FreeRTOS.h"
    #include "task.h"
    #define LORA_TaskDelay(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

class DXLR01 {
    public:
        enum class WorkingMode {
            TRANSMIT, AT
        };

        enum class TransMode {
            TRANSPARENT,
            DIRECTIONAL,
            BROADCAST
        };

        enum class ATCmd {
            AT, RESET, DEFAULT, BAUD, PARI, LEVEL, MODE,
            SLEEP, SWITCH, CHANNEL, MAC, OPENKEY, KEY, PACKET,
            DRSSI, POWE, LBT, LRSSI
        };

        typedef struct {
            uint8_t data[256];
            uint16_t length;
            uint32_t timestamp_ms; 
        } LoraMessage_t;

        using SendCallback = bool (*)(uint8_t* data, uint16_t len, void* userData);
        using ReceiveCallback = bool (*)(uint8_t* data, uint16_t& len, uint32_t timeout, void* userData);

        DXLR01();

        void begin(SendCallback sendCb, ReceiveCallback receiveCb, void* userData);

        bool configure(uint8_t channel = 0x00,
                       uint8_t level = 2,
                       TransMode mode = TransMode::TRANSPARENT,
                       uint16_t address = 0x0001,
                       uint8_t baud = 0);

        bool setMode(WorkingMode mode);
        bool sendAT(ATCmd cmd, const char* arg = nullptr);
        void sendData(const uint8_t *data, size_t len);

        WorkingMode getMode() const { return _mode; }
        TransMode getTransMode() const { return _transMode; }
        uint16_t getAddress() const { return _address; }
        uint8_t getChannel() const { return _channel; }

    private:
        const char* _getATString(ATCmd cmd) const;
        bool _receiveResponse(uint32_t timeoutMs);

        void _delay(uint32_t ms);

        SendCallback _sendCb = nullptr;
        ReceiveCallback _recvCb = nullptr;
        void* _userData = nullptr;

        WorkingMode _mode;
        TransMode _transMode;
        uint16_t _address;
        uint8_t _channel;
};
