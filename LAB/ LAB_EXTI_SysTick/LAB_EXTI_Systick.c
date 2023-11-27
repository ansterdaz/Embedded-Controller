/**
******************************************************************************
* @author  Hanmin Kim
* @Mod		 2023-10-16   	
* @brief   Embedded Controller:  LAB Systick&EXTI with 7 segment
*				
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

int count = 0;
// Initialiization 
void setup(void)
{ 
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	RCC_PLL_init();
	SysTick_init();
	sevensegment_init();
	sevensegment_display_init();

}
// interrupt
void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) {
		 sevensegment_display(0);   // push button to print number 0 
		 count = 0;
		clear_pending_EXTI(BUTTON_PIN); 
	}
}


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		sevensegment_display(count);
		delay_ms(1000);
		count++;
		if (count >9) count =0;
		SysTick_reset();
	}
}

