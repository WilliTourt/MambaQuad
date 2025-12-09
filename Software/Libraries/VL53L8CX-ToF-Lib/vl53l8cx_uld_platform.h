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

#ifndef _VL53L8CX_ULD_PLATFORM_H_
#define _VL53L8CX_ULD_PLATFORM_H_

#include <stdint.h>
#include <string.h>

#include "main.h"

#if !defined(__STM32F1xx_HAL_H) && \
    !defined(__STM32F4xx_HAL_H) && \
    !defined(__STM32F7xx_HAL_H) && \
    !defined(__STM32H7xx_HAL_H) && \
    !defined(__STM32L4xx_HAL_H) && \
    !defined(__STM32G0xx_HAL_H) && \
    !defined(__STM32WBxx_HAL_H) && \
    !defined(__STM32WLxx_HAL_H)
#error "This vl53l8cx library can only be used with STM32Cube HAL drivers"
#endif

#if !defined(HAL_I2C_MODULE_ENABLED) && \
    !defined(HAL_SPI_MODULE_ENABLED)
#error "At least one IIC port or SPI port should be opened"
#endif


typedef enum {
	VL53L8CX_COMMS_I2C,
	VL53L8CX_COMMS_SPI
} VL53L8CX_CommsType;

/**
 * @brief Structure VL53L8CX_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L8CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */
typedef struct {
	#ifdef HAL_I2C_MODULE_ENABLED
	I2C_HandleTypeDef*    hi2c;
	uint8_t  			  i2c_addr;
	#endif

	#ifdef HAL_SPI_MODULE_ENABLED
	SPI_HandleTypeDef*    hspi;
	GPIO_TypeDef*         cs_port;
	uint8_t               cs_pin;
	#endif

	VL53L8CX_CommsType    comms_type;

	GPIO_TypeDef*         reset_port;
	uint8_t               reset_pin;

	GPIO_TypeDef*         interrupt_port;
	uint8_t               interrupt_pin;

	uint8_t               pwr_mode;

	uint8_t               is_drdy;
	uint8_t 			  resolution;
	uint8_t               ranging_freq_hz;
	uint8_t               ranging_mode;

	uint32_t              integrations_period_ms;
	uint8_t               sharpener_percent;
	uint8_t               target_order;

	uint8_t               is_sync_pin_enabled;

	uint32_t              VHV_repeat_cnt;

} VL53L8CX_Platform;

#define VL53L8CX_PLATFORM_OK 			((uint8_t) 0U)
#define VL53L8CX_PLATFORM_TIMEOUT_ERROR ((uint8_t) 1U)
#define VL53L8CX_PLATFORM_INVALID_PARAM ((uint8_t) 127U)
#define VL53L8CX_PLATFORM_STATUS_ERROR 	((uint8_t) 255U)

#define VL53L8CX_COMM_TIMEOUT_MS 100

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#define 	VL53L8CX_NB_TARGET_PER_ZONE		1U

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

// #define 	VL53L8CX_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

// #define VL53L8CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L8CX_DISABLE_NB_SPADS_ENABLED
// #define VL53L8CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L8CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L8CX_DISABLE_RANGE_SIGMA_MM
// #define VL53L8CX_DISABLE_DISTANCE_MM
// #define VL53L8CX_DISABLE_REFLECTANCE_PERCENT
// #define VL53L8CX_DISABLE_TARGET_STATUS
// #define VL53L8CX_DISABLE_MOTION_INDICATOR

/**
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) *p_values : Pointer of value to read.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L8CX_RdByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_value);

/**
 * @brief Mandatory function used to write one single byte.
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) value : Pointer of value to write.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L8CX_WrByte(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t value);

/**
 * @brief Mandatory function used to read multiples bytes.
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to read.
 * @param (uint8_t) *p_values : Buffer of bytes to read.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L8CX_RdMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief Mandatory function used to write multiples bytes.
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of values to write.
 * @param (uint8_t) *p_values : Buffer of bytes to write.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L8CX_WrMulti(
		VL53L8CX_Platform *p_platform,
		uint16_t RegisterAdress,
		uint8_t *p_values,
		uint32_t size);

/**
 * @brief Optional function, only used to perform an hardware reset of the
 * sensor. This function is not used in the API, but it can be used by the host.
 * This function is not mandatory to fill if user don't want to reset the
 * sensor.
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @return (uint8_t) status : 0 if OK
 */
uint8_t VL53L8CX_Reset_Sensor(
		VL53L8CX_Platform *p_platform);

/**
 * @brief Mandatory function, used to swap a buffer. The buffer size is always a
 * multiple of 4 (4, 8, 12, 16, ...).
 * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
 * @param (uint16_t) size : Buffer size to swap
 */
void VL53L8CX_SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size);

/**
 * @brief Mandatory function, used to wait during an amount of time. It must be
 * filled as it's used into the API.
 * @param (VL53L8CX_Platform*) p_platform : Pointer of VL53L8CX platform
 * structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */
uint8_t VL53L8CX_WaitMs(
		VL53L8CX_Platform *p_platform,
		uint32_t TimeMs);

#endif	// _VL53L8CX_ULD_PLATFORM_H_
