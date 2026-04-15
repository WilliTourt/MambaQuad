#include "LoraTask.h"

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
    _serial.send((uint8_t*)data, (uint16_t)len);
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
    memcpy(msg.data, rxData, len);
    msg.timestamp_ms = FreeRTOS::Kernel::getTickCount();

    _toLoraTaskQueue.sendToBack(msg);
}
