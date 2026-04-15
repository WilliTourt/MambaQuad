/**
 * @file dshot.cpp
 * @brief DShot ESC communication class definition
 * 
 * This file contains the implementation of the DShot class methods
 * for interfacing with the DShot ESC.
 * 
 * @author WilliTourt willitourt@foxmail.com
 * @version 1.0
 * @date 2026.04.10
 * 
 * @note Adapted from mokhwasomssi/stm32_hal_dshot on GitHub
 * 
 * @changelog:
 * - (See header file for changes)
 */


#include "dshot.h"

DShot::DShot(TIM_HandleTypeDef *htim, uint32_t channel, DShotType type) :
    _htim(htim),
    _channel(channel),
    _type(type) {}

bool DShot::begin() {
	uint16_t dshot_prescaler;
	uint32_t timer_clock = TIM_CLK_FREQ;

    dshot_prescaler = lrintf((float) timer_clock / static_cast<uint32_t>(_type) + 0.01f) - 1;

    __HAL_TIM_SET_PRESCALER(_htim, dshot_prescaler);
    __HAL_TIM_SET_AUTORELOAD(_htim, MOTOR_BITLENGTH);

    uint16_t dma_id;

    dma_id = _timChannel_to_dmaID(_channel);
    if (dma_id == 0) { return false; }

    _htim->hdma[dma_id]->XferCpltCallback = DShot::_dmaTC_Callback;

    HAL_TIM_PWM_Start(_htim, _channel);

    return true;
}

void DShot::send(uint16_t throttle) {
    _prepareDMABuffer(throttle);

    uint32_t ccrX, ccX;
    switch (_channel) {
        case TIM_CHANNEL_1: ccrX = (uint32_t)&_htim->Instance->CCR1; ccX = TIM_DMA_CC1; break;
        case TIM_CHANNEL_2: ccrX = (uint32_t)&_htim->Instance->CCR2; ccX = TIM_DMA_CC2; break;
        case TIM_CHANNEL_3: ccrX = (uint32_t)&_htim->Instance->CCR3; ccX = TIM_DMA_CC3; break;
        case TIM_CHANNEL_4: ccrX = (uint32_t)&_htim->Instance->CCR4; ccX = TIM_DMA_CC4; break;
        default: return;
    }

    HAL_DMA_Start_IT(_htim->hdma[_timChannel_to_dmaID(_channel)],
                     (uint32_t)_dmabuffer,
                     ccrX,
                     DMA_BUF_SIZE);

    __HAL_TIM_ENABLE_DMA(_htim, ccX);
}

uint16_t DShot::_timChannel_to_dmaID(uint32_t channel) {
    uint16_t dma_id;
    switch (channel) {
        case TIM_CHANNEL_1: dma_id = TIM_DMA_ID_CC1; break;
        case TIM_CHANNEL_2: dma_id = TIM_DMA_ID_CC2; break;
        case TIM_CHANNEL_3: dma_id = TIM_DMA_ID_CC3; break;
        case TIM_CHANNEL_4: dma_id = TIM_DMA_ID_CC4; break;
        default: return 0; // Invalid channel
    }
    return dma_id;
}

uint16_t DShot::_preparePacket(uint16_t value) {
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

void DShot::_prepareDMABuffer(uint16_t value) {
    uint16_t packet = _preparePacket(value);

    for (uint8_t i = 0; i < DSHOT_FRAME_SIZE; i++) {
        _dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    _dmabuffer[16] = 0;
    _dmabuffer[17] = 0;
}

void DShot::_dmaTC_Callback(DMA_HandleTypeDef *hdma) {
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	} else if(hdma == htim->hdma[TIM_DMA_ID_CC2]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	} else if(hdma == htim->hdma[TIM_DMA_ID_CC3]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	} else if(hdma == htim->hdma[TIM_DMA_ID_CC4]) {
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}
