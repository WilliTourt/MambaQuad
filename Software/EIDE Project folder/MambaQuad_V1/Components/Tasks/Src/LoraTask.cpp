#include "LoraTask.h"
#include <cstring>

LoraTask::LoraTask(FreeRTOS::Queue<LoraMessage_t> &fromLoraSerialQueue, LoraSerialTask &serial) :
                   Task(tskIDLE_PRIORITY + 2, 512, "LoRa"),
                   _fromLoraSerialQueue(fromLoraSerialQueue),
                   _serial(serial),
                   _mode(LoRaMode::TRANSMIT),
                   _transMode(LoRaTransMode::TRANSPARENT),
                   _address(0x0001),
                   _channel(0x00) {}

const char* LoraTask::_getATstr(ATcmd cmd) {
    switch (cmd) {
        case ATcmd::AT:         return "AT";
        case ATcmd::RESET:      return "AT+RESET";
        case ATcmd::DEFAULT:    return "AT+DEFAULT";
        case ATcmd::BAUD:       return "AT+BAUD";
        case ATcmd::PARI:       return "AT+PARI";
        case ATcmd::LEVEL:      return "AT+LEVEL";
        case ATcmd::MODE:       return "AT+MODE";
        case ATcmd::SLEEP:      return "AT+SLEEP";
        case ATcmd::SWITCH:     return "AT+SWITCH";
        case ATcmd::CHANNEL:    return "AT+CHANNEL";
        case ATcmd::MAC:        return "AT+MAC";
        case ATcmd::OPENKEY:    return "AT+OPENKEY";
        case ATcmd::KEY:        return "AT+KEY";
        case ATcmd::PACKET:     return "AT+PACKET";
        case ATcmd::DRSSI:      return "AT+DRSSI";
        case ATcmd::POWE:       return "AT+POWE";
        case ATcmd::LBT:        return "AT+LBT";
        case ATcmd::LRSSI:      return "AT+LRSSI";
        default:                return nullptr;
    }
}

bool LoraTask::init() {
    _mode = LoRaMode::TRANSMIT;
    _transMode = LoRaTransMode::TRANSPARENT;
    return true;
}

bool LoraTask::configure(uint8_t channel, uint8_t level, LoRaTransMode mode, 
                         uint16_t address, uint8_t baud) {
    // 保存配置
    _channel = channel;
    _address = address;
    _transMode = mode;
    
    // 进入 AT 模式
    if (!setMode(LoRaMode::AT)) {
        return false;
    }
    
    char arg[16];
    bool ok = true;
    
    // baud rate
    if (baud != 0) {
        snprintf(arg, sizeof(arg), "%d", baud);
        ok &= sendAT(ATcmd::BAUD, arg);
        if (!ok) return false;
    }
    
    // speed level, 0 farthest but slowest, 7 otherwise
    snprintf(arg, sizeof(arg), "%d", level > 7 ? 7 : level);
    ok &= sendAT(ATcmd::LEVEL, arg);
    if (!ok) return false;
    
    // channel
    snprintf(arg, sizeof(arg), "%02X", channel > 0x63 ? 0x63 : channel);
    ok &= sendAT(ATcmd::CHANNEL, arg);
    if (!ok) return false;
    
    // this device's addr
    snprintf(arg, sizeof(arg), "%02X,%02X", (address >> 8) & 0xFF, address & 0xFF);
    ok &= sendAT(ATcmd::MAC, arg);
    if (!ok) return false;
    
    // trans mode (0 transparent, 1 directional, 2 broadcast)
    snprintf(arg, sizeof(arg), "%d", static_cast<int>(mode));
    ok &= sendAT(ATcmd::MODE, arg);
    if (!ok) return false;
    
    ok &= sendAT(ATcmd::RESET, nullptr);
    
    return ok;
}

bool LoraTask::setMode(LoRaMode mode) {
    if (_mode == mode) return true;

    while (_fromLoraSerialQueue.receive(0)) {}
    this->delay(pdMS_TO_TICKS(100));

    const char* cmd = "+++"; // no CRLF
    _serial.send((uint8_t*)cmd, 3);

    auto response = _fromLoraSerialQueue.receive(pdMS_TO_TICKS(500));
    if (!response) return false;

    if (mode == LoRaMode::AT) {
        // "Entry AT"
        if ((response->data[0] == 'E') && (response->data[1] == 'n')) {
            _mode = LoRaMode::AT;
            return true;
        }
    } else {
        // "Exit AT"
        if ((response->data[0] == 'E') && (response->data[1] == 'x')) {
            this->delay(pdMS_TO_TICKS(500)); // wait for reboot
            _mode = LoRaMode::TRANSMIT;
            return true;
        }
    }

    return false;
}

bool LoraTask::sendAT(ATcmd cmd, const char* arg) {
    if (_mode != LoRaMode::AT) return false;

    char buffer[32];
    const char* cmdPrefix = _getATstr(cmd);
    if (cmdPrefix == nullptr) return false;

    if (arg != nullptr) {
        snprintf(buffer, sizeof(buffer), "%s%s\r\n", cmdPrefix, arg);
    } else {
        snprintf(buffer, sizeof(buffer), "%s\r\n", cmdPrefix);
    }

    _serial.send((uint8_t*)buffer, (uint16_t)strlen(buffer));

    auto response = _fromLoraSerialQueue.receive(pdMS_TO_TICKS(500));
    if (!response) return false;

    if (strstr((char*)response->data, "OK") != nullptr) {
        if (cmd == ATcmd::RESET) {
            this->delay(pdMS_TO_TICKS(1000));
            _mode = LoRaMode::TRANSMIT;
        }
        return true;
    }

    return false;
}

void LoraTask::sendData(const uint8_t *data, size_t len) {
    if (_mode != LoRaMode::TRANSMIT) return;
    
    if (_transMode == LoRaTransMode::TRANSPARENT) { // [data]
        _serial.send((uint8_t*)data, (uint16_t)len);
    } else if (_transMode == LoRaTransMode::DIRECTIONAL) { // [addr2][chan1][data]
        uint8_t packet[256];
        if (len + 3 > sizeof(packet)) len = sizeof(packet) - 3;
        
        packet[0] = (_address >> 8) & 0xFF;
        packet[1] = _address & 0xFF;
        packet[2] = _channel;
        memcpy(&packet[3], data, len);
        
        _serial.send(packet, (uint16_t)(len + 3));
    } else if (_transMode == LoRaTransMode::BROADCAST) { // [chan1][data]
        uint8_t packet[256];
        if (len + 1 > sizeof(packet)) len = sizeof(packet) - 1;
        
        packet[0] = _channel;
        memcpy(&packet[1], data, len);
        
        _serial.send(packet, (uint16_t)(len + 1));
    }
}

void LoraTask::taskFunction() {
    for (;;) {
        auto opt = _fromLoraSerialQueue.receive(portMAX_DELAY);
        if (!opt) {
            continue;
        }

        if (_mode == LoRaMode::TRANSMIT) {
            // 透传模式下收到的数据是空中传来的数据包
            // 定点模式下收到的是：[源地址2字节][源信道1字节][数据]
            // 广播模式下收到的是：[源信道1字节][数据]
            
            // TODO: 在这里处理接收到的数据
            // 可以解析后发到飞控任务队列
            // 比如：解析遥控指令、遥测数据等
            
        } else {
            // AT 模式下收到的是命令响应
            // 已经在 sendAT 里处理了，这里可以存日志或做状态机
        }
    }
}



LoraSerialTask::LoraSerialTask(UART_HandleTypeDef *huart,
                               FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                               FreeRTOS::Queue<LoraMessage_t> &toLoraTaskQueue,
                               const char* taskName,
                               UBaseType_t priority,
                               configSTACK_DEPTH_TYPE stackDepth) :
    SerialTaskBase(huart, rxQueue, taskName, priority, stackDepth),
    _toLoraTaskQueue(toLoraTaskQueue) {}

void LoraSerialTask::rxProcess(uint8_t *rxData, uint16_t len) {
    LoraMessage_t msg;
    msg.length = len;
    if (len > sizeof(msg.data)) len = sizeof(msg.data);
    memcpy(msg.data, rxData, len);
    msg.timestamp_ms = FreeRTOS::Kernel::getTickCount();

    _toLoraTaskQueue.sendToBack(msg);
}
