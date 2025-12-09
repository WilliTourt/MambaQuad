

#include "vl53l8cx.h"
#include <cstring>

static VL53L8CX::Status _map_status(uint8_t s) {
    switch(s) {
        case VL53L8CX_STATUS_OK:                return VL53L8CX::Status::OK;
        case VL53L8CX_STATUS_TIMEOUT_ERROR:     return VL53L8CX::Status::TIMEOUT;
        case VL53L8CX_STATUS_CORRUPTED_FRAME:   return VL53L8CX::Status::CORRUPTED_FRAME;
        case VL53L8CX_STATUS_LASER_SAFETY:      return VL53L8CX::Status::LASER_SAFETY;
        case VL53L8CX_STATUS_XTALK_FAILED:      return VL53L8CX::Status::XTALK_FAILED;
        case VL53L8CX_STATUS_FW_CHECKSUM_FAIL:  return VL53L8CX::Status::FW_CHECKSUM_FAIL;
        case VL53L8CX_MCU_ERROR:                return VL53L8CX::Status::MCU_ERROR;
        case VL53L8CX_STATUS_INVALID_PARAM:     return VL53L8CX::Status::INVALID_PARAM;
        default:                                return VL53L8CX::Status::ERROR;
    }
}

#ifdef HAL_I2C_MODULE_ENABLED
VL53L8CX::VL53L8CX(I2C_HandleTypeDef *hi2c, uint8_t i2c_address) {
    std::memset(&_cfg, 0, sizeof(_cfg));
    _cfg.platform.hi2c = hi2c;
    _cfg.platform.i2c_addr = i2c_address;
    _cfg.platform.comms_type = VL53L8CX_COMMS_I2C;
}
#endif

#ifdef HAL_SPI_MODULE_ENABLED
VL53L8CX::VL53L8CX(void *hspi, GPIO_TypeDef *CS_GPIOx, uint8_t CS_Pin) {
    std::memset(&_cfg, 0, sizeof(_cfg));
    _cfg.platform.hspi = (SPI_HandleTypeDef*)hspi;
    _cfg.platform.cs_port = CS_GPIOx;
    _cfg.platform.cs_pin = CS_Pin;
    _cfg.platform.comms_type = VL53L8CX_COMMS_SPI;
}
#endif

VL53L8CX::Status VL53L8CX::_init() {
    uint8_t status = vl53l8cx_init(&_cfg);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::begin() {
    return _init();
}

bool VL53L8CX::isAlive() {
    uint8_t alive = 0;
    uint8_t status = vl53l8cx_is_alive(&_cfg, &alive);
    
    return ((status == VL53L8CX_STATUS_OK) && (alive != 0));
}

VL53L8CX::Status VL53L8CX::setIICAddress(uint16_t i2c_address) {
    uint8_t status = vl53l8cx_set_i2c_address(&_cfg, i2c_address);
    if (status == VL53L8CX_STATUS_OK) {
        _cfg.platform.i2c_addr = (uint8_t)i2c_address;
    }

    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::startRanging() {
    uint8_t status = vl53l8cx_start_ranging(&_cfg);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::stopRanging() {
    uint8_t status = vl53l8cx_stop_ranging(&_cfg);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::_checkDRDY(uint8_t *p_isReady) {
    uint8_t status = vl53l8cx_check_data_ready(&_cfg, p_isReady);
    if (status == VL53L8CX_STATUS_OK) {
        _cfg.platform.is_drdy = *p_isReady;
    }
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::getRangingData(VL53L8CX_ResultsData *p_results) {
    if (p_results == nullptr) { 
        return VL53L8CX::Status::INVALID_PARAM;
    }

    uint8_t status = vl53l8cx_get_ranging_data(&_cfg, &_results);
    if (status == VL53L8CX_STATUS_OK) {
        std::memcpy(p_results, &_results, sizeof(VL53L8CX_ResultsData));
    }
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::readRangingData() {
    uint8_t status = vl53l8cx_get_ranging_data(&_cfg, &_results);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::refreshSettings() {
    uint8_t status = VL53L8CX_STATUS_OK;
    uint8_t tmp8 = 0;
    uint32_t tmp32 = 0;

    status |= vl53l8cx_get_resolution(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.resolution = tmp8;

    status |= vl53l8cx_get_ranging_frequency_hz(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.ranging_freq_hz = tmp8;

    status |= vl53l8cx_get_integration_time_ms(&_cfg, &tmp32);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.integrations_period_ms = tmp32;

    status |= vl53l8cx_get_sharpener_percent(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.sharpener_percent = tmp8;

    status |= vl53l8cx_get_target_order(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.target_order = tmp8;

    status |= vl53l8cx_get_ranging_mode(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.ranging_mode = tmp8;

    status |= vl53l8cx_get_external_sync_pin_enable(&_cfg, &tmp8);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.is_sync_pin_enabled = tmp8;

    status |= vl53l8cx_get_VHV_repeat_count(&_cfg, &tmp32);
    if (status == VL53L8CX_STATUS_OK) _cfg.platform.VHV_repeat_cnt = tmp32;

    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::dciReadData(uint8_t *data, uint32_t index, uint16_t data_size) {
    uint8_t status = vl53l8cx_dci_read_data(&_cfg, data, index, data_size);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::dciWriteData(uint8_t *data, uint32_t index, uint16_t data_size) {
    uint8_t status = vl53l8cx_dci_write_data(&_cfg, data, index, data_size);
    return _map_status(status);
}

VL53L8CX::Status VL53L8CX::dciReplaceData(uint8_t *data, uint32_t index, uint16_t data_size,
                                          uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos) {
    uint8_t status = vl53l8cx_dci_replace_data(&_cfg, data, index, data_size,
                                               new_data, new_data_size, new_data_pos);
    return _map_status(status);
}
