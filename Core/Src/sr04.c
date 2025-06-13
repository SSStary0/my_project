#include "sr04.h"
#include "pid.h"


void RCCdelay_us(uint32_t udelay) {
__IO uint32_t Delay = udelay * 72 / 8;
do {
__NOP();
} while (Delay--);
}

void GET_Distance()
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
	RCCdelay_us(12);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
}
