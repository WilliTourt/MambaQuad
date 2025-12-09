/**
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "vl53l8cx_uld_platform.h"

uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {

	if (p_platform == NULL || p_value == NULL) return VL53L8CX_PLATFORM_INVALID_PARAM;

	switch (p_platform->comms_type) {
		case VL53L8CX_COMMS_I2C: {
		#if defined(HAL_I2C_MODULE_ENABLED)

			HAL_StatusTypeDef hal_ret = HAL_I2C_Mem_Read(p_platform->hi2c,
							p_platform->i2c_addr,
							RegisterAdress,
							I2C_MEMADD_SIZE_16BIT,
							p_value,
							1,
							VL53L8CX_COMM_TIMEOUT_MS);

			if (hal_ret == HAL_OK) { return VL53L8CX_PLATFORM_OK; }
			else if (hal_ret == HAL_TIMEOUT) { return VL53L8CX_PLATFORM_TIMEOUT_ERROR; }
			else { return VL53L8CX_PLATFORM_STATUS_ERROR; }
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		case VL53L8CX_COMMS_SPI: {
		#if defined(HAL_SPI_MODULE_ENABLED)

			HAL_StatusTypeDef hal_ret;
			if (p_platform->hspi == NULL || p_platform->cs_port == NULL) return VL53L8CX_PLATFORM_INVALID_PARAM;

			uint8_t addr_buf[2];
			addr_buf[0] = (uint8_t)((RegisterAdress >> 8) & 0xFF);
			addr_buf[1] = (uint8_t)(RegisterAdress & 0xFF);

			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_RESET);
			hal_ret = HAL_SPI_Transmit(p_platform->hspi, addr_buf, 2, VL53L8CX_COMM_TIMEOUT_MS);
			if (hal_ret != HAL_OK) {
				HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);
				return (hal_ret == HAL_TIMEOUT) ? VL53L8CX_PLATFORM_TIMEOUT_ERROR : VL53L8CX_PLATFORM_STATUS_ERROR;
			}

			hal_ret = HAL_SPI_Receive(p_platform->hspi, p_value, 1, VL53L8CX_COMM_TIMEOUT_MS);
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);

			if (hal_ret == HAL_OK) return VL53L8CX_PLATFORM_OK;
			if (hal_ret == HAL_TIMEOUT) return VL53L8CX_PLATFORM_TIMEOUT_ERROR;
			return VL53L8CX_PLATFORM_STATUS_ERROR;
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		default: return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

}

uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {

	if (p_platform == NULL) {
		return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

	switch (p_platform->comms_type) {
		case VL53L8CX_COMMS_I2C: {
		#if defined(HAL_I2C_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret = HAL_I2C_Mem_Write(p_platform->hi2c,
							 p_platform->i2c_addr,
							 RegisterAdress,
							 I2C_MEMADD_SIZE_16BIT,
							 &value,
							 1,
							 VL53L8CX_COMM_TIMEOUT_MS);

			if (hal_ret == HAL_OK) { return VL53L8CX_PLATFORM_OK; }
			else if (hal_ret == HAL_TIMEOUT) { return VL53L8CX_PLATFORM_TIMEOUT_ERROR; }
			else { return VL53L8CX_PLATFORM_STATUS_ERROR; }
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		case VL53L8CX_COMMS_SPI: {
		#if defined(HAL_SPI_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret;
			if (p_platform->hspi == NULL || p_platform->cs_port == NULL) return VL53L8CX_PLATFORM_INVALID_PARAM;

			uint8_t addr_buf[2];
			addr_buf[0] = (uint8_t)((RegisterAdress >> 8) & 0xFF);
			addr_buf[1] = (uint8_t)(RegisterAdress & 0xFF);

			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_RESET);
			hal_ret = HAL_SPI_Transmit(p_platform->hspi, addr_buf, 2, VL53L8CX_COMM_TIMEOUT_MS);
			if (hal_ret != HAL_OK) {
				HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);
				return (hal_ret == HAL_TIMEOUT) ? VL53L8CX_PLATFORM_TIMEOUT_ERROR : VL53L8CX_PLATFORM_STATUS_ERROR;
			}

			hal_ret = HAL_SPI_Transmit(p_platform->hspi, &value, 1, VL53L8CX_COMM_TIMEOUT_MS);
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);

			if (hal_ret == HAL_OK) return VL53L8CX_PLATFORM_OK;
			if (hal_ret == HAL_TIMEOUT) return VL53L8CX_PLATFORM_TIMEOUT_ERROR;
			return VL53L8CX_PLATFORM_STATUS_ERROR;
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		default: return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

}

uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {

	if (p_platform == NULL || p_values == NULL || size == 0) {
		return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

	switch (p_platform->comms_type) {
		case VL53L8CX_COMMS_I2C: {
		#if defined(HAL_I2C_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret = HAL_I2C_Mem_Write(p_platform->hi2c,
							 p_platform->i2c_addr,
							 RegisterAdress,
							 I2C_MEMADD_SIZE_16BIT,
							 p_values,
							 (uint16_t)size,
							 VL53L8CX_COMM_TIMEOUT_MS);

			if (hal_ret == HAL_OK) { return VL53L8CX_PLATFORM_OK; }
			else if (hal_ret == HAL_TIMEOUT) { return VL53L8CX_PLATFORM_TIMEOUT_ERROR; }
			else { return VL53L8CX_PLATFORM_STATUS_ERROR; }
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		case VL53L8CX_COMMS_SPI: {
		#if defined(HAL_SPI_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret;
			if (p_platform->hspi == NULL || p_platform->cs_port == NULL) return VL53L8CX_PLATFORM_INVALID_PARAM;

			uint8_t addr_buf[2];
			addr_buf[0] = (uint8_t)((RegisterAdress >> 8) & 0xFF);
			addr_buf[1] = (uint8_t)(RegisterAdress & 0xFF);

			/* Pull CS low */
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_RESET);

			/* Send address */
			hal_ret = HAL_SPI_Transmit(p_platform->hspi, addr_buf, 2, VL53L8CX_COMM_TIMEOUT_MS);
			if (hal_ret != HAL_OK) {
				HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);
				return (hal_ret == HAL_TIMEOUT) ? VL53L8CX_PLATFORM_TIMEOUT_ERROR : VL53L8CX_PLATFORM_STATUS_ERROR;
			}

			/* Send payload */
			hal_ret = HAL_SPI_Transmit(p_platform->hspi, p_values, (uint16_t)size, VL53L8CX_COMM_TIMEOUT_MS);

			/* Release CS */
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);

			if (hal_ret == HAL_OK) return VL53L8CX_PLATFORM_OK;
			if (hal_ret == HAL_TIMEOUT) return VL53L8CX_PLATFORM_TIMEOUT_ERROR;
			return VL53L8CX_PLATFORM_STATUS_ERROR;
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		default: return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

}

uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {

	if (p_platform == NULL || p_values == NULL || size == 0) {
		return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

	switch (p_platform->comms_type) {
		case VL53L8CX_COMMS_I2C: {
		#if defined(HAL_I2C_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret = HAL_I2C_Mem_Read(p_platform->hi2c,
							 p_platform->i2c_addr,
							 RegisterAdress,
							 I2C_MEMADD_SIZE_16BIT,
							 p_values,
							 (uint16_t)size,
							 VL53L8CX_COMM_TIMEOUT_MS);

			if (hal_ret == HAL_OK) { return VL53L8CX_PLATFORM_OK; }
			else if (hal_ret == HAL_TIMEOUT) { return VL53L8CX_PLATFORM_TIMEOUT_ERROR; }
			else { return VL53L8CX_PLATFORM_STATUS_ERROR; }
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		case VL53L8CX_COMMS_SPI: {
		#if defined(HAL_SPI_MODULE_ENABLED)
		
			HAL_StatusTypeDef hal_ret;
			if (p_platform->hspi == NULL || p_platform->cs_port == NULL) return VL53L8CX_PLATFORM_INVALID_PARAM;

			uint8_t addr_buf[2];
			addr_buf[0] = (uint8_t)((RegisterAdress >> 8) & 0xFF);
			addr_buf[1] = (uint8_t)(RegisterAdress & 0xFF);

			/* Pull CS low */
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_RESET);

			/* Send address */
			hal_ret = HAL_SPI_Transmit(p_platform->hspi, addr_buf, 2, VL53L8CX_COMM_TIMEOUT_MS);
			if (hal_ret != HAL_OK) {
				HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);
				return (hal_ret == HAL_TIMEOUT) ? VL53L8CX_PLATFORM_TIMEOUT_ERROR : VL53L8CX_PLATFORM_STATUS_ERROR;
			}

			/* Receive payload */
			hal_ret = HAL_SPI_Receive(p_platform->hspi, p_values, (uint16_t)size, VL53L8CX_COMM_TIMEOUT_MS);

			/* Release CS */
			HAL_GPIO_WritePin(p_platform->cs_port, (uint16_t)p_platform->cs_pin, GPIO_PIN_SET);

			if (hal_ret == HAL_OK) return VL53L8CX_PLATFORM_OK;
			if (hal_ret == HAL_TIMEOUT) return VL53L8CX_PLATFORM_TIMEOUT_ERROR;
			return VL53L8CX_PLATFORM_STATUS_ERROR;
		
		#else
			return VL53L8CX_PLATFORM_INVALID_PARAM;
		#endif
		}

		default: return VL53L8CX_PLATFORM_INVALID_PARAM;
	}

}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform) {
	uint8_t status = 0;
	
	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */
	
	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO to LOW */
	/* Set pin CORE_1V8 to LOW */
	VL53L8CX_WaitMs(p_platform, 100);

	
	/* Set pin LPN to HIGH */
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	VL53L8CX_WaitMs(p_platform, 100);

	return status;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs) {
	HAL_Delay(TimeMs);
	return 0;
}
