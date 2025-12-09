#ifndef __ICM42688_H__
#define __ICM42688_H__

#include "cpp_main.h"

#include "ICM42688_reg.h"

#ifdef HAL_I2C_MODULE_ENABLED
#include "i2c.h"
#endif
#ifdef HAL_SPI_MODULE_ENABLED
#include "spi.h"
#endif


class ICM42688
{
    public:
        enum class ICM42688_StatusTypeDef : uint8_t
        {
            OK = 0,
            ERROR,
        };

        #ifdef HAL_I2C_MODULE_ENABLED
        ICM42688(I2C_HandleTypeDef* hi2c,
             uint8_t address = ICM42688_I2C_ADDR_LOW
            );
        #endif

        #ifdef HAL_SPI_MODULE_ENABLED
        ICM42688(SPI_HandleTypeDef* hspi,
             GPIO_TypeDef* cs_port,
             uint16_t cs_pin
            );
        #endif 

        ICM42688_StatusTypeDef begin(void);

        ICM42688_StatusTypeDef enable_acc(uint8_t odr = ICM42688_ACCEL_ODR_100HZ,
                                         uint8_t fs = ICM42688_ACCEL_FS_4G,
                                         uint8_t acc_mode = ICM42688_ACCEL_MODE_LN,
                                         uint8_t UIF_bw = ICM42688_ACCEL_UIF_BW_50Hz,
                                         uint8_t M2F_enable = ICM42688_ACCEL_M2F_CLOSE,
                                         uint8_t AAF_enable = ICM42688_ACCEL_AAF_CLOSE,
                                         );
        ICM42688_StatusTypeDef disable_acc(void);
        ICM42688_StatusTypeDef enable_gyro(uint8_t odr = ICM42688_GYRO_ODR_100HZ,
                                          uint8_t fs = ICM42688_GYRO_FS_2000DPS,
                                          uint8_t gyro_mode = ICM42688_GYRO_MODE_LN,
                                          uint8_t UIF_bw = ICM42688_GYRO_UIF_BW_50Hz,
                                          uint8_t M2F_enable = ICM42688_GYRO_M2F_CLOSE,
                                          uint8_t NF_enable = ICM42688_GYRO_NF_CLOSE,
                                          uint8_t AAF_enable = ICM42688_GYRO_GAF_CLOSE,
                                          );
        ICM42688_StatusTypeDef disable_gyro(void);
        ICM42688_StatusTypeDef enable_temp(uint8_t DLPF_bw = ICM42688_TEMP_DLPF_BW_50Hz);
        ICM42688_StatusTypeDef disable_temp(void);

        ICM42688_StatusTypeDef get_ax(int16_t* ax);
        ICM42688_StatusTypeDef get_ay(int16_t* ay);
        ICM42688_StatusTypeDef get_az(int16_t* az);
        ICM42688_StatusTypeDef get_acc(int16_t acc[3]);
        ICM42688_StatusTypeDef get_gx(int16_t* gx);
        ICM42688_StatusTypeDef get_gy(int16_t* gy);
        ICM42688_StatusTypeDef get_gz(int16_t* gz);
        ICM42688_StatusTypeDef get_gyro(int16_t gyro[3]);
        ICM42688_StatusTypeDef get_temp(int16_t* temp);
        ICM42688_StatusTypeDef get_accgyro(int16_t acc[3], int16_t gyro[3]);
        ICM42688_StatusTypeDef get_all(int16_t acc[3], int16_t gyro[3], int16_t* temp);

    private:

        class ICM42688_FIFO
        {
            public:
                enum class ICM42688_FIFO_StatusTypeDef : uint8_t
                {
                    OK = 0,
                    ERROR,
                };

                ICM42688_FIFO(ICM42688* parent)
                    : _parent(parent)
                {}

                ICM42688_FIFO_StatusTypeDef enable(uint8_t fifo_mode = ICM42688_FIFO_MODE_STREAM,
                                                uint8_t acc_enable = ICM42688_FIFO_ACC_ENABLE,
                                                uint8_t gyro_enable = ICM42688_FIFO_GYRO_ENABLE,
                                                uint8_t temp_enable = ICM42688_FIFO_TEMP_DISABLE,
                                                uint8_t timestamp_enable = ICM42688_FIFO_TIMESTAMP_DISABLE);
                ICM42688_FIFO_StatusTypeDef disable(void);
                ICM42688_FIFO_StatusTypeDef read(uint8_t* pdata, uint16_t len);
                ICM42688_FIFO_StatusTypeDef get_count(uint16_t* count);
        }

        class ICM42688_INT
        {
            public:
                enum class ICM42688_INT_StatusTypeDef : uint8_t
                {
                    OK = 0,
                    ERROR,
                };

                enum class INT_CLEAR_MODE : uint8_t
                {
                    ICM42688_INT_CLEAR_ON_STATUS_BIT_READ = 0,
                    ICM42688_INT_CLEAR_ON_REG_READ = 1,
                    ICM42688_INT_CLEAR_ON_STATUS_BIT_AND_REG_READ = 2,
                };

                enum class INT_LEVEL : uint8_t
                {
                    ICM42688_INT_LEVEL_ACTIVE_LOW = 0,
                    ICM42688_INT_LEVEL_ACTIVE_HIGH = 1,
                };

                enum class INT_DRIVE : uint8_t
                {
                    ICM42688_INT_DRIVE_OPEN_DRAIN = 0,
                    ICM42688_INT_DRIVE_PUSH_PULL = 1,
                };

                enum class INT_MODE : uint8_t
                {
                    ICM42688_INT_PULSE = 0,
                    ICM42688_INT_LATCHED = 1,
                };

                enum class INT_SOURCE : uint8_t
                {
                    ICM42688_INT_DISABLE = 0,
                    ICM42688_INT_ENABLE = 1,
                };

                ICM42688_INT(ICM42688* parent,
                            GPIO_TypeDef* int1_port,
                            uint16_t int1_pin,
                            GPIO_TypeDef* int2_port,
                            uint16_t int2_pin);

                ICM42688_INT(ICM42688* parent,
                            GPIO_TypeDef* int1_port,
                            uint16_t int1_pin);

                ICM42688_INT_StatusTypeDef begin(void);

                ICM42688_INT_StatusTypeDef set_int_latchClear_mode(uint8_t DRDY_clear_mode = INT_CLEAR_MODE::ICM42688_INT_CLEAR_ON_STATUS_BIT_READ,
                                                           uint8_t FIFO_THS_clear_mode = INT_CLEAR_MODE::ICM42688_INT_CLEAR_ON_STATUS_BIT_READ,
                                                           uint8_t FIFO_FULL_clear_mode = INT_CLEAR_MODE::ICM42688_INT_CLEAR_ON_STATUS_BIT_READ);  

                ICM42688_INT_StatusTypeDef set_int1_pin_cfg(uint8_t level = INT_LEVEL::ICM42688_INT_LEVEL_ACTIVE_HIGH,
                                                           uint8_t drive = INT_DRIVE::ICM42688_INT_DRIVE_PUSH_PULL,
                                                           uint8_t mode = INT_MODE::ICM42688_INT_LATCHED);
                ICM42688_INT_StatusTypeDef set_int2_pin_cfg(uint8_t level = INT_LEVEL::ICM42688_INT_LEVEL_ACTIVE_HIGH,
                                                           uint8_t drive = INT_DRIVE::ICM42688_INT_DRIVE_PUSH_PULL,
                                                           uint8_t mode = INT_MODE::ICM42688_INT_LATCHED);
                ICM42688_INT_StatusTypeDef set_int1_source(uint8_t DRDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t FIFO_THS_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t FIFO_FULL_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t UI_FSYNC_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t PLL_RDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t RESET_DONE_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t UI_AGC_RDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t I3C_ERROR_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t SMD_enable = INT_SOURCE::ICM42688_INT_DISABLE, 
                                                    uint8_t WOM_Z_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t WOM_Y_enable = INT_SOURCE::ICM42688_INT_DISABLE, 
                                                    uint8_t WOM_X_enable = INT_SOURCE::ICM42688_INT_DISABLE);          
                ICM42688_INT_StatusTypeDef set_int2_source(uint8_t DRDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t FIFO_THS_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t FIFO_FULL_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t UI_FSYNC_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t PLL_RDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t RESET_DONE_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t UI_AGC_RDY_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t I3C_ERROR_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t SMD_enable = INT_SOURCE::ICM42688_INT_DISABLE, 
                                                    uint8_t WOM_Z_enable = INT_SOURCE::ICM42688_INT_DISABLE,
                                                    uint8_t WOM_Y_enable = INT_SOURCE::ICM42688_INT_DISABLE, 
                                                    uint8_t WOM_X_enable = INT_SOURCE::ICM42688_INT_DISABLE);                      
        }


    protected:
        void icm_delay(uint32_t ms);
        ICM42688_StatusTypeDef writeReg(uint8_t reg, uint8_t* pdata, uint16_t len);
        ICM42688_StatusTypeDef readReg(uint8_t reg, uint8_t* pdata, uint16_t len);

        #ifdef HAL_I2C_MODULE_ENABLED
        I2C_HandleTypeDef* _hi2c;
        uint8_t _address;
        #endif

        #ifdef HAL_SPI_MODULE_ENABLED
        SPI_HandleTypeDef* _hspi;
        GPIO_TypeDef* _cs_port;
        uint16_t _cs_pin;
        #endif

        bool _is_right_address = 1;

};


#endif
