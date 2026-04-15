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




/* Lora

// H

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


        enum class LoRaTansmissionMode {
            TRANSPARENT,    // 透传
            DIRECTIONAL,     // 定向
            BROADCAST        // 广播
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

        uint16_t _address;
        uint8_t _channel;
};


// cpp


LoraTask::LoraTask(FreeRTOS::Queue<LoraMessage_t> &fromLoraSerialQueue, LoraSerialTask &serial) :
                   Task(tskIDLE_PRIORITY + 2, 512, "LoRa"),
                   _fromLoraSerialQueue(fromLoraSerialQueue),
                   _serial(serial),
                   _mode(LoRaMode::TRANSMIT) {}

const char* LoraTask::_getATstr(ATcmd cmd) {
    switch (cmd) {
        case ATcmd::AT:         return "AT\r\n";
        case ATcmd::RESET:      return "AT+RESET\r\n";
        case ATcmd::DEFAULT:    return "AT+DEFAULT\r\n";
        case ATcmd::BAUD:       return "AT+BAUD\r\n";
        case ATcmd::PARI:       return "AT+PARI\r\n";
        case ATcmd::HELP:       return "AT+HELP\r\n";
        case ATcmd::LEVEL:      return "AT+LEVEL\r\n";
        case ATcmd::MODE:       return "AT+MODE\r\n";
        case ATcmd::SLEEP:      return "AT+SLEEP\r\n";
        case ATcmd::SWITCH:     return "AT+SWITCH\r\n";
        case ATcmd::CHANNEL:    return "AT+CHANNEL\r\n";
        case ATcmd::MAC:        return "AT+MAC\r\n";
        case ATcmd::OPENKEY:    return "AT+OPENKEY\r\n";
        case ATcmd::KEY:        return "AT+KEY\r\n";
        case ATcmd::PACKET:     return "AT+PACKET\r\n";
        case ATcmd::DRSSI:      return "AT+DRSSI\r\n";
        case ATcmd::POWE:       return "AT+POWE\r\n";
        case ATcmd::LBT:        return "AT+LBT\r\n";
        case ATcmd::LRSSI:      return "AT+LRSSI\r\n";
        case ATcmd::ERSSI:      return "AT+ERSSI\r\n";
        default:                return "ERR";
    }
}

bool LoraTask::init() {
    // placeholder initialization; nothing to configure at the moment
    // could reset queues or send an AT command to probe the module
    _mode = LoRaMode::TRANSMIT;
    return true;
}

void LoraTask::setMode(LoRaMode mode) {
    const char* enterCmd = "+++\r\n";
    _serial.send((uint8_t*)enterCmd, (uint16_t)strlen(enterCmd));

    _mode = mode;
}

bool LoraTask::sendAT(ATcmd cmd, uint8_t *response) {
    const char* at = _getATstr(cmd);
    if (at == nullptr || strcmp(at, "ERR") == 0) {
        return false;
    }

    _serial.send((uint8_t*)at, (uint16_t)strlen(at));
    // response handling can be added later using `response` pointer
    (void)response;
    return true;
}

void LoraTask::sendData(const uint8_t *data, size_t len) {
    // when in transmit mode send raw bytes to the radio
    if(_mode != LoRaMode::TRANSMIT) 
        return;

    _serial.send((uint8_t*)data, (uint16_t)len);
}




/
 * mode: 0-透传，1-定向，2-广播
 * address: 定向模式下的目标地址，广播模式下无效
 * channel: 信道号
 * tansLevel: 传输等级，范围0-7，数值越大传输距离更短，但比特率更高
 * 
 /
void LoraTask::set_transmissionMode(LoRaTansmissionMode mode, uint16_t address, uint8_t channel, uint8_t tansLevel) {

    if(_mode != LoRaMode::AT) {
        // Must be in AT mode to configure transmission mode
        return;
    }

    setMode(LoRaMode::AT); // Ensure we're in AT mode to send configuration commands
    char at[64];

    _address = address;     // 需要保存下来,因为后续发送数据时需要知道目标地址
    _channel = channel;

    switch (mode) {
        case LoRaTansmissionMode::TRANSPARENT:
            // Configure for transparent transmission
            sprintf(at, "AT+MODE=0\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+LEVEL=%d\r\n", tansLevel);
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+CHANNEL=%d\r\n", channel);
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+RESET\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));

            break;
        case LoRaTansmissionMode::DIRECTIONAL:
            // Configure for directional transmission
            sprintf(at, "AT+MODE=1\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+LEVEL=%d\r\n", tansLevel);
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+RESET\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));

            break;
        case LoRaTansmissionMode::BROADCAST:
            // Configure for broadcast transmission
            sprintf(at, "AT+MODE=2\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+LEVEL=%d\r\n", tansLevel);
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));
            sprintf(at, "AT+RESET\r\n");
            _serial.send((uint8_t*)at, (uint16_t)strlen(at));

            break;
    }

    setMode(LoRaMode::AT);
}



void LoraTask::taskFunction() {
    for (;;) {
        auto opt = _fromLoraSerialQueue.receive(portMAX_DELAY);
        if (!opt) {
            continue;
        }

        // LoraMessage_t msg = *opt;

        // msg.data contains a line-terminated sequence from the serial peripheral
        if (_mode == LoRaMode::TRANSMIT) {
            // when in transmit mode, received messages are packets coming in
            // from the radio module.  dispatch them to whoever needs the data.
            // e.g. push into another queue or invoke a callback.
        } else {
            // in AT mode the bytes are command responses; could be parsed
            // by a state machine or stored in a buffer for later retrieval.
        }
    }
}

*/