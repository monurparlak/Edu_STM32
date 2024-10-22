/* ************************************************************************************************************************
 * ************************************************************************************************************************
 * Bare Metal (Low Layer/Register) / Standard Peripheral Library (SPL) / Hardware Abstraction Layer Library (HAL)
 * By Mustafa Onur Parlak
 * GitHub Link: https://github.com/legarthos
 * ***********************************************************************************************************************/
/* ************************************************************************************************************************
 * Project Purpose:
 * It contains the basic notes and contents I took while learning STM32 with LL-SPL-HAL DRIVER.
 * General Draft Code Style with Bare-Metal / Low Layer Library / Register
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
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void delay(uint32_t time);
void PLL_CLOCK_REGISTER();
void GPIO_OUTPUT_REGISTER();

int main(void) {
	PLL_CLOCK_REGISTER();
	GPIO_OUTPUT_REGISTER();

	while (1) {
		GPIOD->ODR = 0x0000F000; // OUTPUT DATA REGISTER Set
		delay(16800000);

		GPIOD->ODR = 0x00000000; //OUTPUT DATA REGISTER Reset
		delay(33600000);
	}
}

void PLL_CLOCK_REGISTER() {
	RCC->CR |= 0x00010000;		    // HSEON (High Speed External) ENABLE
	while(!(RCC->CR & 0x00020000));	// HSEON (High Speed External) FLAG Bekliyor
	RCC->CR |= 0x00080000;			// CSS (Clock System Security) ENABLE

	RCC->PLLCFGR |= 0x00400000;		// PLL (Phase Locked Loop) HSE Se�ilir
	RCC->PLLCFGR |= 0x00000004;		// PLL M = 4
	RCC->PLLCFGR |= 0x00005A00;		// Pll N = 168
	RCC->PLLCFGR |= 0x00000000;		// PLL p = 2

	RCC->CFGR |= 0x00000000; 		// AHB Prescaler = 1
	RCC->CFGR |= 0x00080000; 		// APB2 Prescaler = 2
	RCC->CFGR |= 0x00001400; 		// APB1 Prescaler = 4
	RCC->CIR  |= 0x00800000; 		// CSS (Clock System Security) FLAG Temizlenir

}

void GPIO_OUTPUT_REGISTER() {
	RCC->AHB1ENR = 0x00000008;      // GPIOD portu ENABLE

	GPIOD->MODER = 0x55000000;      // Kart �st�ndeki Ledler OUTPUT
	GPIOD->PUPDR = 0x00000000;      // NOPULL
	GPIOD->OTYPER = 0x00000000;     // PUSH PULL
	GPIOD->OSPEEDR = 0xFF000000;    // 100 MHz
}

void delay(uint32_t time) {
	while(time--);
}

extern "C" void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {return;}
extern "C" uint16_t EVAL_AUDIO_GetSampleCallBack(void) {return -1;}
