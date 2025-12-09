

#pragma once

extern "C" {

#include "vl53l8cx_uld_platform.h"
#include "vl53l8cx_api.h"

}



class VL53L8CX {
    public:

        enum class Status : uint8_t {
            OK = VL53L8CX_STATUS_OK,
            TIMEOUT = VL53L8CX_STATUS_TIMEOUT_ERROR,
            CORRUPTED_FRAME = VL53L8CX_STATUS_CORRUPTED_FRAME,
            LASER_SAFETY = VL53L8CX_STATUS_LASER_SAFETY,
            XTALK_FAILED = VL53L8CX_STATUS_XTALK_FAILED,
            FW_CHECKSUM_FAIL = VL53L8CX_STATUS_FW_CHECKSUM_FAIL,
            MCU_ERROR = VL53L8CX_MCU_ERROR,
            INVALID_PARAM = VL53L8CX_STATUS_INVALID_PARAM,
            ERROR = VL53L8CX_STATUS_ERROR
        };

        #ifdef HAL_I2C_MODULE_ENABLED
        VL53L8CX(I2C_HandleTypeDef *hi2c, uint8_t i2c_address = VL53L8CX_DEFAULT_I2C_ADDRESS);
        #endif
        #ifdef HAL_SPI_MODULE_ENABLED
        VL53L8CX(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIOx, uint8_t CS_Pin);
        #endif

        Status begin();

        bool isAlive();
        Status setIICAddress(uint16_t i2c_address);

        Status startRanging();
        Status stopRanging();

        /* Communication/read helpers: call underlying C API and update internal cache */
        Status getRangingData(VL53L8CX_ResultsData *p_results); /* fills internal _results and immediately get it */
        Status readRangingData(); /* only updates internal _results */
        Status refreshSettings(); /* updates platform fields (resolution, freq, integration time, etc.) */

        inline uint8_t  getResolution() const         { return _cfg.platform.resolution;                }
        inline uint8_t  getRangingFrequencyHz() const { return _cfg.platform.ranging_freq_hz;           }
        inline uint8_t  getRangingMode() const        { return _cfg.platform.ranging_mode;              }
        inline uint32_t getIntegrationTimeMs() const  { return _cfg.platform.integrations_period_ms;    }
        inline uint8_t  getSharpenerPercent() const   { return _cfg.platform.sharpener_percent;         }
        inline uint8_t  getTargetOrder() const        { return _cfg.platform.target_order;              }
        inline bool     isSyncPinEnabled() const      { return (bool)_cfg.platform.is_sync_pin_enabled; }
        inline uint32_t getVHVRepeatCount() const     { return _cfg.platform.VHV_repeat_cnt;            }
        inline bool     isDataReady() const           { return (bool)_cfg.platform.is_drdy;             }

        inline const VL53L8CX_ResultsData& results() const { return _results; }

        Status dciReadData(uint8_t *data, uint32_t index, uint16_t data_size);
        Status dciWriteData(uint8_t *data, uint32_t index, uint16_t data_size);
        Status dciReplaceData(uint8_t *data, uint32_t index, uint16_t data_size,
                              uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);

        VL53L8CX_Configuration *getConfig() { return &_cfg; }

    private:
        VL53L8CX_Configuration _cfg;
        VL53L8CX_ResultsData _results;

        Status _init();
        Status _checkDRDY(uint8_t *p_isReady);

};
