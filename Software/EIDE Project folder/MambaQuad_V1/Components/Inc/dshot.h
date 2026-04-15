/**
 * @file dshot.h
 * @brief DShot ESC communication class definition
 * 
 * This header defines the DShot class for interfacing with DShot ESCs
 * using STM32 timers and DMA.
 * 
 * @author WilliTourt willitourt@foxmail.com
 * @version 1.0
 * @date 2026.04.10
 * 
 * @note Adapted from mokhwasomssi/stm32_hal_dshot on GitHub
 * 
 * @changelog:
 * - 2026.04.10: Initial release, not tested yet.
 * 
 */


#include "tim.h"
#include <cmath>



#define TIM_CLK_FREQ            168000000UL // 168 MHz

#define DSHOT1200_HZ    		24000000UL
#define DSHOT600_HZ     		12000000UL
#define DSHOT300_HZ     		6000000UL
#define DSHOT150_HZ     		3000000UL

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

class DShot {
    public:

        enum class DShotType : uint32_t {
            DSHOT150 = DSHOT150_HZ,
            DSHOT300 = DSHOT300_HZ,
            DSHOT600 = DSHOT600_HZ,
            DSHOT1200 = DSHOT1200_HZ
        };

        DShot(TIM_HandleTypeDef *htim, uint32_t channel, DShotType type);

        bool begin();
        void send(uint16_t throttle);

    private:

        uint16_t _timChannel_to_dmaID(uint32_t channel);

        void _prepareDMABuffer(uint16_t value);
        uint16_t _preparePacket(uint16_t value);

        static void _dmaTC_Callback(DMA_HandleTypeDef *hdma);

        TIM_HandleTypeDef *_htim;
        uint32_t _channel;
        DShotType _type;

        static constexpr uint16_t DMA_BUF_SIZE = DSHOT_DMA_BUFFER_SIZE;
        uint32_t _dmabuffer[DMA_BUF_SIZE];
};
