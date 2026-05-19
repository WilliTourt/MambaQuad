#pragma once

#include <FreeRTOS/Task.hpp>
#include <FreeRTOS/Queue.hpp>

#include "main.h"
#include "data_types.h"

extern "C" {
    #include "w25qxx.h"
}
// #include "w25qxx.h"

class FDRTask : public FreeRTOS::Task {
    public:
        FDRTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

        bool init();

    private:
        void taskFunction() override;

        bool _read(uint32_t address, uint8_t *buf, uint32_t len);
        bool _write(uint32_t address, uint8_t *buf, uint32_t len);
        bool _erase(uint32_t address, uint32_t len);
        bool _eraseChip();

        SPI_HandleTypeDef *_hspi;
        GPIO_TypeDef *cs_port;
        uint16_t cs_pin;
        W25QXX_HandleTypeDef _w25qxx;

        FDRHeader_t _header;
};
