#include "FDRTask.h"
#include "DBGTask.h"
#include <stdio.h>

FDRTask::FDRTask(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) :
    Task(tskIDLE_PRIORITY + 1, 2048, "FDR"),
    _hspi(hspi),
    cs_port(cs_port),
    cs_pin(cs_pin) {}

bool FDRTask::init() {
    if (w25qxx_init(&_w25qxx, _hspi, cs_port, cs_pin) != W25QXX_Ok) {
        return false;
    }

    uint8_t buf[sizeof(FDRHeader_t)] = {0};
    _read(0, buf, sizeof(buf));

    memcpy(&_header, buf, sizeof(_header));

    if (_header.magic != 0x4D425144) { // MBQD
        // If new chip or first boot: init header
        _header.magic = 0x4D425144;
        _header.pwr_on_cnt = 1;
        _header.total_flight_sec = 0;
        _header.total_flight_meters = 0;
        _header.log_rate_Hz = 10;
        _header.write_ptr = 0x001000;

        memcpy(buf, &_header, sizeof(_header));
        _erase(0, 4096);
        _write(0, buf, sizeof(_header));
        return true;
    }

    _header.pwr_on_cnt++;
    memcpy(buf, &_header, sizeof(_header));
    _erase(0, 4096);
    _write(0, buf, sizeof(_header));

    return true;
}

bool FDRTask::_read(uint32_t address, uint8_t *buf, uint32_t len) {
    return (w25qxx_read(&_w25qxx, address, buf, len) == W25QXX_Ok) ? true : false;
}

bool FDRTask::_write(uint32_t address, uint8_t *buf, uint32_t len) {
    return (w25qxx_write(&_w25qxx, address, buf, len) == W25QXX_Ok) ? true : false;
}

bool FDRTask::_erase(uint32_t address, uint32_t len) {
    return (w25qxx_erase(&_w25qxx, address, len) == W25QXX_Ok) ? true : false;
}

bool FDRTask::_eraseChip() {
    return (w25qxx_chip_erase(&_w25qxx) == W25QXX_Ok) ? true : false;
}

void FDRTask::taskFunction() {

    static char msg[128];

    this->delay(2000);
    snprintf(msg, sizeof(msg), "FDR Task started. Magic=%08X, Power on count=%ld",
             _header.magic, _header.pwr_on_cnt);
    DBGQ.sendToBack((uint8_t*)msg);

    for (;;) {
        this->delay(5000);
        DBGQ.sendToBack((uint8_t*)"FDR: Idle...");
    }
}