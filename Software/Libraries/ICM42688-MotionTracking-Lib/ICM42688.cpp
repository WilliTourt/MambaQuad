#include "ICM42688.h"

//===========================================================================================================

ICM42688::ICM42688(I2C_HandleTypeDef* hi2c,
                    uint8_t address)
    : _hi2c(hi2c), _address(address)
{
    if(_address != ICM42688_I2C_ADDR_LOW && _address != ICM42688_I2C_ADDR_HIGH)
        _is_right_address = 0; // 错误地址

}

ICM42688::ICM42688_StatusTypeDef ICM42688::WriteReg(uint8_t reg, uint8_t* pdata, uint16_t len)
{
    if(HAL_I2C_Mem_Write(_hi2c, _address << 1, reg, I2C_MEMADD_SIZE_8BIT, pdata, len, 1000) != HAL_OK)
    {
        return ICM42688_StatusTypeDef::ERROR;
    }
    return ICM42688_StatusTypeDef::OK;
}

ICM42688::ICM42688_StatusTypeDef ICM42688::ReadReg(uint8_t reg, uint8_t* pdata, uint16_t len)
{
    if(HAL_I2C_Mem_Read(_hi2c, _address << 1, reg, I2C_MEMADD_SIZE_8BIT, pdata, len, 1000) != HAL_OK)
    {
        return ICM42688_StatusTypeDef::ERROR;
    }
    return ICM42688_StatusTypeDef::OK;
}

void ICM42688::ICM_Delay(uint32_t ms)
{
    HAL_Delay(ms);
}

//===========================================================================================================

ICM42688::ICM42688_StatusTypeDef ICM42688::Init(void)
{
    uint8_t *data;
    ICM42688_StatusTypeDef res;

    if(!_is_right_address)
        return ICM42688_StatusTypeDef::ERROR;

    *data = 0x01;
    res = WriteReg(REG_DEVICE_CONFIG, data, 1); // 软复位
    if(res != ICM42688_StatusTypeDef::OK) return res;
    ICM_Delay(10);

    res = ReadReg(REG_WHO_AM_I, data, 1); // 验证ID
    if(res != ICM42688_StatusTypeDef::OK) return res;


    return res;
}