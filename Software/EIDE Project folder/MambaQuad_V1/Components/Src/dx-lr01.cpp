#include "dx-lr01.h"
#include <cstring>
#include <cstdio>

DXLR01::DXLR01() :
    _sendCb(nullptr),
    _recvCb(nullptr),
    _userData(nullptr),
    _mode(WorkingMode::TRANSMIT),
    _transMode(TransMode::TRANSPARENT),
    _address(0x0001),
    _channel(0x00) {}

void DXLR01::begin(SendCallback sendCb, ReceiveCallback receiveCb, void* userData) {
    _sendCb = sendCb;
    _recvCb = receiveCb;
    _userData = userData;
}

void DXLR01::_delay(uint32_t ms) {
    #if LORA_USE_FREERTOS == 1
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            LORA_TaskDelay(ms);
        } else {
            HAL_Delay(ms);
        }
    #else
        HAL_Delay(ms);
    #endif
}


const char* DXLR01::_getATString(ATCmd cmd) const {
    switch (cmd) {
        case ATCmd::AT:      return "AT";
        case ATCmd::RESET:   return "AT+RESET";
        case ATCmd::DEFAULT: return "AT+DEFAULT";
        case ATCmd::BAUD:    return "AT+BAUD";
        case ATCmd::PARI:    return "AT+PARI";
        case ATCmd::LEVEL:   return "AT+LEVEL";
        case ATCmd::MODE:    return "AT+MODE";
        case ATCmd::SLEEP:   return "AT+SLEEP";
        case ATCmd::SWITCH:  return "AT+SWITCH";
        case ATCmd::CHANNEL: return "AT+CHANNEL";
        case ATCmd::MAC:     return "AT+MAC";
        case ATCmd::OPENKEY: return "AT+OPENKEY";
        case ATCmd::KEY:     return "AT+KEY";
        case ATCmd::PACKET:  return "AT+PACKET";
        case ATCmd::DRSSI:   return "AT+DRSSI";
        case ATCmd::POWE:    return "AT+POWE";
        case ATCmd::LBT:     return "AT+LBT";
        case ATCmd::LRSSI:   return "AT+LRSSI";
        default:             return nullptr;
    }
}

bool DXLR01::_receiveResponse(uint32_t timeoutMs) {
    if (_recvCb == nullptr) return false;
    
    uint8_t buffer[256];
    uint16_t len = 0;
    
    if (_recvCb(buffer, len, timeoutMs, _userData)) {
        return (strstr((char*)buffer, "OK") != nullptr);
    }
    return false;
}

bool DXLR01::setMode(WorkingMode mode) {
    if (_mode == mode) return true;
    if (_sendCb == nullptr) return false;

    // 清空待处理的响应
    if (_recvCb != nullptr) {
        uint8_t dummy[256];
        uint16_t dummyLen = 0;
        while (_recvCb(dummy, dummyLen, 0, _userData)) {}
    }
    
    _delay(100);

    const char* cmd = "+++";
    _sendCb((uint8_t*)cmd, 3, _userData);

    uint8_t response[256];
    uint16_t respLen = 0;
    
    if (_recvCb == nullptr || !_recvCb(response, respLen, 500, _userData)) {
        return false;
    }

    if (mode == WorkingMode::AT) {
        if ((response[0] == 'E') && (response[1] == 'n')) { // Entry AT
            _mode = WorkingMode::AT;
            return true;
        }
    } else {
        if ((response[0] == 'E') && (response[1] == 'x')) { //Exit AT
            _delay(500);
            _mode = WorkingMode::TRANSMIT;
            return true;
        }
    }

    return false;
}

bool DXLR01::sendAT(ATCmd cmd, const char* arg) {
    if (_mode != WorkingMode::AT) return false;
    if (_sendCb == nullptr) return false;

    char buffer[32];
    const char* cmdPrefix = _getATString(cmd);
    if (cmdPrefix == nullptr) return false;

    if (arg != nullptr) {
        snprintf(buffer, sizeof(buffer), "%s%s\r\n", cmdPrefix, arg);
    } else {
        snprintf(buffer, sizeof(buffer), "%s\r\n", cmdPrefix);
    }

    _sendCb((uint8_t*)buffer, (uint16_t)strlen(buffer), _userData);
    
    bool ok = _receiveResponse(500);
    if (ok && cmd == ATCmd::RESET) {
        _delay(1000);
        _mode = WorkingMode::TRANSMIT;
    }
    return ok;
}

/**
 * @brief 配置 LoRa 模块参数
 * @param channel 信道号 (0x00-0x63)
 * @param level 速率等级 0-7 (0最慢最远, 7最快最近)
 * @param mode 传输模式
 * @param address 本机地址 (定点模式时需要)
 * @param baud 波特率代码 (3=9600, 7=115200, 0=不修改)
 * @return true 配置成功
 */
bool DXLR01::configure(uint8_t channel, uint8_t level, TransMode mode,
                       uint16_t address, uint8_t baud) {
    _channel = channel;
    _address = address;
    _transMode = mode;

    if (!setMode(WorkingMode::AT)) {
        return false;
    }

    char arg[16];
    bool ok = true;

    if (baud != 0) {
        snprintf(arg, sizeof(arg), "%d", baud);
        ok &= sendAT(ATCmd::BAUD, arg);
        if (!ok) return false;
    }

    snprintf(arg, sizeof(arg), "%d", level > 7 ? 7 : level);
    ok &= sendAT(ATCmd::LEVEL, arg);
    if (!ok) return false;

    snprintf(arg, sizeof(arg), "%02X", channel > 0x63 ? 0x63 : channel);
    ok &= sendAT(ATCmd::CHANNEL, arg);
    if (!ok) return false;

    snprintf(arg, sizeof(arg), "%02X,%02X", (address >> 8) & 0xFF, address & 0xFF);
    ok &= sendAT(ATCmd::MAC, arg);
    if (!ok) return false;

    snprintf(arg, sizeof(arg), "%d", static_cast<int>(mode));
    ok &= sendAT(ATCmd::MODE, arg);
    if (!ok) return false;

    ok &= sendAT(ATCmd::RESET, nullptr);
    return ok;
}

void DXLR01::sendData(const uint8_t *data, size_t len) {
    if (_mode != WorkingMode::TRANSMIT) return;
    if (_sendCb == nullptr) return;

    if (_transMode == TransMode::TRANSPARENT) {
        _sendCb((uint8_t*)data, (uint16_t)len, _userData);
    } else if (_transMode == TransMode::DIRECTIONAL) {
        uint8_t packet[256];
        if (len + 3 > sizeof(packet)) len = sizeof(packet) - 3;

        packet[0] = (_address >> 8) & 0xFF;
        packet[1] = _address & 0xFF;
        packet[2] = _channel;
        memcpy(&packet[3], data, len);

        _sendCb(packet, (uint16_t)(len + 3), _userData);
    } else if (_transMode == TransMode::BROADCAST) {
        uint8_t packet[256];
        if (len + 1 > sizeof(packet)) len = sizeof(packet) - 1;

        packet[0] = _channel;
        memcpy(&packet[1], data, len);

        _sendCb(packet, (uint16_t)(len + 1), _userData);
    }
}
