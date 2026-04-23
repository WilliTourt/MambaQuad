#include "LoraTask.h"
#include <cstring>

LoraTask::LoraTask(LoraSerialTask &serial,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &fromLoraSerialQueue,
    FreeRTOS::Queue<DXLR01::LoraMessage_t> &loraQueue) :
    Task(tskIDLE_PRIORITY + 2, 512, "LoRa"),
    _serial(serial),
    _fromLoraSerialQueue(fromLoraSerialQueue),
    _loraQueue(loraQueue) {}

bool LoraTask::init(uint8_t channel, uint8_t level, DXLR01::TransMode mode,
                    uint16_t address, uint8_t baud) {
    
    this->delay(pdMS_TO_TICKS(200));

    _lora.begin(loraSendCb, loraReceiveCb, this);

    // 信道 0x01, 速率等级 2, 透传模式, 地址 0x0001, 波特率 7=115200: 0x01, 2, DXLR01::TransMode::TRANSPARENT, 0x0001, 7
    bool ok = _lora.configure(channel, level, mode, address, baud);
    
    return ok;
}

void LoraTask::taskFunction() {
    for (;;) {
        auto opt = _fromLoraSerialQueue.receive(portMAX_DELAY);
        if (!opt) {
            continue;
        }

        if (_lora.getMode() == DXLR01::WorkingMode::TRANSMIT) {
            switch (_lora.getTransMode()) {
                case DXLR01::TransMode::TRANSPARENT:
                    // 透传模式下收到的数据是空中传来的数据包
                    break;
                case DXLR01::TransMode::DIRECTIONAL:
                    // 定点模式下收到的是：[源地址2字节][源信道1字节][数据]
                    break;
                case DXLR01::TransMode::BROADCAST:
                    // 广播模式下收到的是：[源信道1字节][数据]
                    break;
            }
            
            // TODO: 在这里处理接收到的数据 fromLoraSerialQueue
            // 可以解析后发到飞控任务队列 loraqueue
            // 比如：解析遥控指令、遥测数据等
            
        } else {
            // AT 模式下收到的是命令响应
            // 已经在 sendAT 里处理了，这里可以存日志或做状态机
        }
    }
}



LoraSerialTask::LoraSerialTask(UART_HandleTypeDef *huart,
                               FreeRTOS::Queue<SerialTaskBase::RxPacket> &rxQueue,
                               FreeRTOS::Queue<DXLR01::LoraMessage_t> &toLoraTaskQueue,
                               const char* taskName,
                               UBaseType_t priority,
                               configSTACK_DEPTH_TYPE stackDepth) :
    SerialTaskBase(huart, rxQueue, taskName, priority, stackDepth),
    _toLoraTaskQueue(toLoraTaskQueue) {}

void LoraSerialTask::rxProcess(uint8_t *rxData, uint16_t len) {
    DXLR01::LoraMessage_t msg;
    msg.length = len;
    if (len > sizeof(msg.data)) len = sizeof(msg.data);
    memcpy(msg.data, rxData, len);
    msg.timestamp_ms = FreeRTOS::Kernel::getTickCount();

    _toLoraTaskQueue.sendToBack(msg);
}
