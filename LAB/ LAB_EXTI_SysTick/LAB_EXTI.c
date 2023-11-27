/**
******************************************************************************
* @author  Hanmin Kim
* @Mod		 2023-10-16   	
* @brief   Embedded Controller:  LAB EXTI with 7 segment
*				
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"


#define LED_PIN 	5
#define BUTTON_PIN 13

unsigned int cnt = 0;


void setup(void)
{
	RCC_PLL_init();
	sevensegment_display_init();
	// Priority Highest(0) External Interrupt 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
}

int main(void) {
	setup();
	while (1) {}
}

//EXTI for Pin 13
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		
		sevensegment_display(cnt % 10);
		cnt++; 
		if (cnt > 9) cnt = 0;
		for(int i = 0; i < 500000;i++){}
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


