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
FreeRTOS::Queue<GPSData_t> gpsQueue(1);

FreeRTOS::Queue<SerialTaskBase::RxPacket> usart2Queue(1);
FreeRTOS::Queue<SerialTaskBase::RxPacket> usart4Queue(1);
FreeRTOS::Queue<GPSData_t> gpsSerialQueue(1);

BlinkTask blinkTask;
DBGTask dbgTask(imuQueue, magQueue, baroQueue, gpsQueue);

IMUTask imuTask(&hspi1, ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, imuQueue);
MagTask magTask(&hi2c1, QMC5883P::QMC5883P_Mode::NORMAL, QMC5883P::QMC5883P_Spd::ODR_100HZ, magQueue);
BaroTask baroTask(&hi2c2, ICP10111::ICP10111_MeasurementMode::LOW_NOISE, baroQueue);

GPSSerialTask gpsSerialTask(&huart4, usart4Queue, gpsSerialQueue);
GPSTask gpsTask(gpsSerialTask, gpsSerialQueue, gpsQueue);

MotorTask motor1(&htim8, TIM_CHANNEL_1, DShot::DShotType::DSHOT600);

int cpp_main() {

	if (imuTask.init() &&
		baroTask.init() &&
		magTask.init() &&
		gpsSerialTask.init() &&
		motor1.init()
	) {
		HAL_GPIO_WritePin(LED_SENS_GPIO_Port, LED_SENS_Pin, GPIO_PIN_RESET);
	}

	FreeRTOS::Kernel::startScheduler();

	while (1);
	return 0;
}
