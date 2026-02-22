/**
 * @file cpp_main.cpp
 * @version 
 * 
 * @brief 
 * 
 * @CHANGELOG:
 * 
 * 
 */

#include "cpp_main.h"
#include "tasks.h"

FreeRTOS::Queue<IMUData_t> imuQueue(1);
FreeRTOS::Queue<BaroData_t> baroQueue(1);
FreeRTOS::Queue<MagData_t> magQueue(1);

BlinkTask blinkTask;
DBGTask dbgTask(imuQueue, magQueue, baroQueue);

IMUTask imuTask(&hspi1, ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, imuQueue);
MagTask magTask(&hi2c1, QMC5883P::QMC5883P_Mode::NORMAL, QMC5883P::QMC5883P_Spd::ODR_100HZ, magQueue);
BaroTask baroTask(&hi2c2, ICP10111::ICP10111_MeasurementMode::LOW_NOISE, baroQueue);

int cpp_main() {
	// HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_SET);
	// HAL_Delay(100);
	// HAL_GPIO_WritePin(BUZ_GPIO_Port, BUZ_Pin, GPIO_PIN_RESET);

	if (imuTask.init() && baroTask.init() && magTask.init()) {
		HAL_GPIO_WritePin(LED_SENS_GPIO_Port, LED_SENS_Pin, GPIO_PIN_RESET);
	}

	FreeRTOS::Kernel::startScheduler();

	while (1);
	return 0;
}
