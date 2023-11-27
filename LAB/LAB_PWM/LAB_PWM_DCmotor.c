/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Hanmin Kim
Created          : 05-03-2021
Modified         : 10-15-2023
Language/ver     : C++ in Keil uVision

Description      : LAB_PWM DC motor
/----------------------------------------------------------------*/

#include "stm32f411xe.h"
#include "math.h"

// #include "ecSTM32F411.h"
#include "ecPinNames.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"   
#include "ecSysTick.h"
#include "ecEXTI.h"
// Definition Button Pin & PWM Port, Pin
#define BUTTON_PIN 13
#define LED_PIN   5
#define PWM_PIN PA_0
#define DIR_PIN 2


uint32_t _count = 0;
uint32_t pwmwidth = 0;
uint32_t flag = 1;

void setup(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void LED_toggle(void){
GPIOA-> ODR^=1<<5;
}

int main(void) {
   // Initialization --------------------------------------------------
   setup();   
   
   // Infinite Loop ---------------------------------------------------
   while(1){
   
		  PWM_duty(PWM_PIN, (0.25+0.5*pwmwidth)*flag);
   }
}
// Initialiization 
void setup(void) {   
   RCC_PLL_init();	
   SysTick_init();
      
   // PWM of 20 msec:  TIM2_CH1 (PA_5 AFmode)
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);
   GPIO_pupd(GPIOC, BUTTON_PIN, EC_PU);
	
	 GPIO_init(GPIOC, DIR_PIN,OUTPUT);
	 GPIO_pupd(GPIOC, DIR_PIN, EC_PU);
	 GPIO_otype(GPIOC, DIR_PIN, EC_PUSH_PULL);
	 GPIO_ospeed(GPIOC, DIR_PIN,EC_FAST);
	
	 GPIO_write(GPIOC,DIR_PIN,LOW);
   EXTI_init(GPIOC, BUTTON_PIN, FALL,0);
   
   PWM_init(PWM_PIN);
   PWM_period(PWM_PIN,1);   // 20 msec PWM period
  
   TIM_UI_init(TIM3, 1);         // TIM3 Update-Event Interrupt every 1 msec 
   TIM_UI_enable(TIM3);
}

void EXTI15_10_IRQHandler(void){
   if (is_pending_EXTI(BUTTON_PIN)){
 
     flag ^= 1; // motor on, off
		 
      clear_pending_EXTI(BUTTON_PIN);
      
   }
}      

void TIM3_IRQHandler(void){ 
   
   if(is_UIF(TIM3)){         // Check UIF(update interrupt flag)
      _count++; 
      
      if (_count>2000)         // every 2sec , velocity change
      {     
        pwmwidth ^=1;
				_count =0;
      }
      clear_UIF(TIM3);       // Clear UI flag by writing 0
   }
}
