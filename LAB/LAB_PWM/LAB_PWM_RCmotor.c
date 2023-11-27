/**
******************************************************************************
* @author  Hanmin Kim
* @Mod		 2023-10-16   	
* @brief   Embedded Controller:  LAB Timer RC motor  
*				
* 
******************************************************************************
*/



#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"   // ecPWM2.h
#include "ecEXTI.h"
// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13

#define PWM_PIN PA_1

uint32_t _count = 0;
uint32_t pwmwidth = 0;

void setup(void);
//void TIM3_IRQHandler(void);


void LED_toggle(void){
GPIOA-> ODR^=1<<5;
}

int main(void) {
	// Initialization --------------------------------------------------
	setup();	
	
	// Infinite Loop ---------------------------------------------------
	while(1){
	
	}
}


// Initialiization 
void setup(void) {	
	RCC_PLL_init();
	SysTick_init();
		
	// PWM of 20 msec:  TIM2_CH1 (PA_5 AFmode)
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
	
	PWM_init(PWM_PIN);
	PWM_period(PWM_PIN,20);   // 20 msec PWM period
	
	TIM_UI_init(TIM3, 1);			// TIM2 Update-Event Interrupt every 1 msec 
	TIM_UI_enable(TIM3);
}

void EXTI15_10_IRQHandler(void){
	if (is_pending_EXTI(BUTTON_PIN)){
	
		pwmwidth = 0;
		PWM_duty(PWM_PIN,(( 0.5+(0.11*pwmwidth))/20) );
		clear_pending_EXTI(BUTTON_PIN);
	}
}		

void TIM3_IRQHandler(void){ 
	
	if(is_UIF(TIM3)){			// Check UIF(update interrupt flag)
		_count++; 
		
		if (_count>500) 
		{
			PWM_duty(PWM_PIN,(( 0.5+(0.11*pwmwidth))/20) );
			pwmwidth++;
			if(pwmwidth>18){ pwmwidth=0;}
			
			_count = 0;
		}
		clear_UIF(TIM3); 		// Clear UI flag by writing 0
	}
}
