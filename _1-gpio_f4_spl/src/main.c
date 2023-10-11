/* ************************************************************************************************************************
 * ************************************************************************************************************************
 * Standard Peripheral Library (SPL) / Hardware Abstraction Layer Library (HAL)
 * By Mustafa Onur Parlak
 *
 * GitHub Link: https://github.com/legarthos
 * ***********************************************************************************************************************/
/* ************************************************************************************************************************
 * Project Purpose:
 * It contains the basic notes and contents I took while learning STM32 with LL-SPL-HAL DRIVER.
 * General Draft Code Style with Standard Peripheral Library
 *
 
 * ************************************************************************************************************************
 * Peripherals to be used:
 * GPIO (General Purpose Input Output)
 *
 * ************************************************************************************************************************
 * Update Time and Reason:
 * 2022.28.10 - Initialize
 *
 * ************************************************************************************************************************/
#include <stddef.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

GPIO_InitTypeDef GPIO_InitStruct;

void GPIO_Configuration();
void delay(uint32_t time);

int main(void) {
	GPIO_Configuration();

	while (1) {
		GPIO_SetBits(GPIOC, GPIO_Pin_All);
		delay(16800000);

		GPIO_ResetBits(GPIOC, GPIO_Pin_All);
		delay(16800000);
	}
}

void GPIO_Configuration() {
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void delay(uint32_t time) {
	while(time--);
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  return -1;
}
