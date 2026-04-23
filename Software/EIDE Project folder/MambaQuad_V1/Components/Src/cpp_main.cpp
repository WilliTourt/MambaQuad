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

#include "DBGTask.h"

FreeRTOS::Queue<IMUData_t> imuQueue(1);
FreeRTOS::Queue<BaroData_t> baroQueue(1);
FreeRTOS::Queue<MagData_t> magQueue(1);
FreeRTOS::Queue<GPSData_t> gpsQueue(1);
FreeRTOS::Queue<DXLR01::LoraMessage_t> loraQueue(1);

FreeRTOS::Queue<SerialTaskBase::RxPacket> usart2Queue(1);
FreeRTOS::Queue<SerialTaskBase::RxPacket> usart4Queue(1);
FreeRTOS::Queue<GPSData_t> gpsSerialQueue(1);
FreeRTOS::Queue<DXLR01::LoraMessage_t> loraSerialQueue(1);


BlinkTask blinkTask;
DBGTask dbgTask(imuQueue, magQueue, baroQueue, gpsQueue, DBGQ);

IMUTask imuTask(&hspi1, ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, imuQueue);
MagTask magTask(&hi2c1, QMC5883P::QMC5883P_Mode::NORMAL, QMC5883P::QMC5883P_Spd::ODR_100HZ, magQueue);
BaroTask baroTask(&hi2c2, ICP10111::ICP10111_MeasurementMode::LOW_NOISE, baroQueue);

GPSSerialTask gpsSerialTask(&huart4, usart4Queue, gpsSerialQueue);
GPSTask gpsTask(gpsSerialTask, gpsSerialQueue, gpsQueue);

// LoraSerialTask loraSerialTask(&huart2, usart2Queue, loraSerialQueue);
// LoraTask loraTask(loraSerialTask, loraSerialQueue, loraQueue);

MotorTask motor1(&htim8, TIM_CHANNEL_1, DShot::DShotType::DSHOT600);
MotorTask motor2(&htim8, TIM_CHANNEL_2, DShot::DShotType::DSHOT600);
MotorTask motor3(&htim8, TIM_CHANNEL_3, DShot::DShotType::DSHOT600);
MotorTask motor4(&htim8, TIM_CHANNEL_4, DShot::DShotType::DSHOT600);

int cpp_main() {

	if (imuTask.init() &&
		baroTask.init() &&
		magTask.init() &&
		gpsSerialTask.init()
	) {
		HAL_GPIO_WritePin(LED_SENS_GPIO_Port, LED_SENS_Pin, GPIO_PIN_RESET); // turn on sensor LED
	}

	if (motor1.init() &&
		motor2.init() &&
		motor3.init() &&
		motor4.init()
	) {
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET); // turn off error LED
	}

	FreeRTOS::Kernel::startScheduler();

	while (1);
	return 0;
}
