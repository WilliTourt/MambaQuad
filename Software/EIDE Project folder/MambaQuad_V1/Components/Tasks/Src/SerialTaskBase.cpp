#include "SerialTaskBase.h"
#include <cstring>

SerialTaskBase* SerialTaskBase::_instances[5] = {nullptr};

SerialTaskBase::SerialTaskBase(UART_HandleTypeDef *huart,
                               FreeRTOS::Queue<RxPacket> &rxQueue,
                               const char* taskName,
                               UBaseType_t priority,
                               configSTACK_DEPTH_TYPE stackDepth) :
    Task(priority, stackDepth, taskName),
    _huart(huart),
    _rxQueue(rxQueue)
    /* ,rxByte(0) */ {
    if (huart == &huart1) { _instances[0] = this; }
    else if (huart == &huart2) { _instances[1] = this; }
    else if (huart == &huart3) { _instances[2] = this; }
    else if (huart == &huart4) { _instances[3] = this; }
    else if (huart == &huart5) { _instances[4] = this; }
}

bool SerialTaskBase::init() {
    // return (HAL_UART_Receive_IT(_huart, &_rxByte, 1) == HAL_OK);
    return (HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rxBuffer, RX_BUF_SIZE) == HAL_OK);
}

bool SerialTaskBase::send(uint8_t* data, uint16_t len) {
    return (HAL_UART_Transmit(_huart, data, len, 100) == HAL_OK);
}

void SerialTaskBase::taskFunction() {
    for (;;) {
        auto packet = _rxQueue.receive(portMAX_DELAY);
        if (packet) {
            rxProcess(packet->data, packet->length);
        }
    }
}

// void SerialTaskBase::irqHandler(UART_HandleTypeDef *huart) {
//     SerialTaskBase* instance = nullptr;
//     if (huart == &huart1) { instance = _instances[0]; }
//     else if (huart == &huart2) { instance = _instances[1]; }
//     else if (huart == &huart3) { instance = _instances[2]; }
//     else if (huart == &huart4) { instance = _instances[3]; }
//     else if (huart == &huart5) { instance = _instances[4]; }
//     else return;

//     bool higherPriorityTaskWoken = pdFALSE;
//     instance->_rxQueue.sendToBackFromISR(higherPriorityTaskWoken, instance->_rxByte);

//     HAL_UART_Receive_IT(huart, &instance->_rxByte, 1);

//     portYIELD_FROM_ISR(higherPriorityTaskWoken);
// }

void SerialTaskBase::rxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    SerialTaskBase* instance = nullptr;
    if (huart == &huart1) instance = _instances[0];
    else if (huart == &huart2) { instance = _instances[1]; }
    else if (huart == &huart3) { instance = _instances[2]; }
    else if (huart == &huart4) { instance = _instances[3]; }
    else if (huart == &huart5) { instance = _instances[4]; }
    else return;

    if (instance) {
        if (Size == 0) { return; }

        RxPacket packet;
        packet.length = Size;
        memcpy(packet.data, instance->_rxBuffer, Size);

        bool higherPriorityTaskWoken = pdFALSE;
        instance->_rxQueue.sendToBackFromISR(higherPriorityTaskWoken, packet);

        HAL_UARTEx_ReceiveToIdle_DMA(instance->_huart, instance->_rxBuffer, RX_BUF_SIZE);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     SerialTaskBase::irqHandler(huart);
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    SerialTaskBase::rxEventCallback(huart, Size);
}
