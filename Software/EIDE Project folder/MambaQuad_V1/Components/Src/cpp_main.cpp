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
#include "dshot.h"

#include "DBGTask.h"

FreeRTOS::Queue<IMUData_t> imuQueue(4);
FreeRTOS::Queue<BaroData_t> baroQueue(2);
FreeRTOS::Queue<MagData_t> magQueue(2);
FreeRTOS::Queue<GPSData_t> gpsQueue(1);
FreeRTOS::Queue<DXLR01::LoraMessage_t> loraQueue(1);

FreeRTOS::Queue<SerialTaskBase::RxPacket> usart2Queue(1);
FreeRTOS::Queue<SerialTaskBase::RxPacket> usart4Queue(1);
FreeRTOS::Queue<GPSData_t> gpsSerialQueue(1);
FreeRTOS::Queue<DXLR01::LoraMessage_t> loraSerialQueue(1);
FreeRTOS::Queue<ControlData_t> ctrlQueue(1);
FreeRTOS::Queue<AttitudeData_t> attQueue(2);


BlinkTask blinkTask;
DBGTask dbgTask(imuQueue, magQueue, baroQueue, gpsQueue, DBGQ, ctrlQueue, attQueue);

IMUTask imuTask(&hspi1, ICM42688P_CS_GPIO_Port, ICM42688P_CS_Pin, imuQueue);
MagTask magTask(&hi2c1, QMC5883P::QMC5883P_Mode::NORMAL, QMC5883P::QMC5883P_Spd::ODR_100HZ, magQueue);
BaroTask baroTask(&hi2c2, ICP10111::ICP10111_MeasurementMode::LOW_NOISE, baroQueue);
AttitudeTask attTask(imuQueue, magQueue, attQueue);

// GPSSerialTask gpsSerialTask(&huart4, usart4Queue, gpsSerialQueue);
// GPSTask gpsTask(gpsSerialTask, gpsSerialQueue, gpsQueue);

// LoraSerialTask loraSerialTask(&huart2, usart2Queue, loraSerialQueue);
// LoraTask loraTask(loraSerialTask, loraSerialQueue, loraQueue, ctrlQueue, attQueue);

DShot m1(&htim8, TIM_CHANNEL_1, DShot::DShotType::DSHOT600);
DShot m2(&htim8, TIM_CHANNEL_2, DShot::DShotType::DSHOT600);
DShot m3(&htim8, TIM_CHANNEL_3, DShot::DShotType::DSHOT600);
DShot m4(&htim8, TIM_CHANNEL_4, DShot::DShotType::DSHOT600);
ControlTask ctrl(m1, m2, m3, m4, &ctrlQueue, &attQueue);

FDRTask fdr(&hspi2, FLASH_CS_GPIO_Port, FLASH_CS_Pin);

void beep() {
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 500);
	HAL_Delay(150);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim1, 500);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
	HAL_Delay(150);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim1, 333);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 166);
	HAL_Delay(150);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

extern "C" void dshot_send_all() {
	ControlTask::send();
}

int cpp_main() {

	if (!fdr.init()) {
		HAL_GPIO_WritePin(LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET); // turn on error LED
	}

	if (imuTask.init() &&
		baroTask.init() &&
		magTask.init()
		// gpsSerialTask.init() &&
		// loraSerialTask.init()
	) {
		HAL_GPIO_WritePin(LED_SENS_GPIO_Port, LED_SENS_Pin, GPIO_PIN_RESET); // turn on sensor LED
	}

	ctrl.init();
	// loraTask.init(0,0,DXLR01::TransMode::TRANSPARENT,0,0);

	beep();
	FreeRTOS::Kernel::startScheduler();

	while (1);
	return 0;
}
