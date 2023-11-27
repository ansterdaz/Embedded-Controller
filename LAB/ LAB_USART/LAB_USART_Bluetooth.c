/**
******************************************************************************
* @author   2023-11-15 by Hanmin
* @brief   Embedded Controller:  LAB - USART
*
******************************************************************************
*/



#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"
#include "ecPWM.h"
#include "ecTIM.h"


#define PWM_PIN_1 PA_0 
#define PWM_PIN_2 PA_1
#define L_DIR 2
#define R_DIR 3

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
static volatile float Duty_1 = 0;
static volatile float Duty_2 = 0;


uint8_t PC_string[]="Loop:\r\n";

void setup(void){
	RCC_PLL_init();
	SysTick_init();
	
	// USART2: USB serial init
	UART2_init();
	UART2_baud(BAUD_38400);

	// USART1: BT serial init 
	UART1_init();
	UART1_baud(BAUD_9600);
	
	//PWM init
	PWM_init(PWM_PIN_1);
	PWM_init(PWM_PIN_2);
	PWM_period(PWM_PIN_1,1);
	PWM_period(PWM_PIN_2,1);
	
	GPIO_init(GPIOC,L_DIR,OUTPUT);
	GPIO_write(GPIOC,L_DIR,LOW);
	GPIO_init(GPIOC,R_DIR,OUTPUT);
  GPIO_write(GPIOC,R_DIR,LOW);
	
}

int main(void){	
	setup();
	printf("MCU Initialized\r\n");	
	while(1){
		
		PWM_duty(PWM_PIN_1,Duty_1); 
		PWM_duty(PWM_PIN_2,Duty_2);
		
	
		USART2_write(PC_string, 7);
		delay_ms(2000);        
	}
}

void USART1_IRQHandler(){          		
	if(is_USART1_RXNE()){
		PC_Data = USART1_read();		
    USART_write(USART1, &PC_Data,1);
    
		if(PC_Data == 'W'){       //forward
			Duty_1=0.8; 
		  Duty_2=0.8;
		  
	}
  else if(PC_Data == 'S'){  //stop
		
	    Duty_1=0; 
		  Duty_2=0;
	}
	else if(PC_Data == 'A'){  //left 
      Duty_1=0.8; 
		  Duty_2=0.5;
  }		
  else if(PC_Data =='D') {  //right
		  Duty_1=0.5; 
		  Duty_2=0.8;
	}
	
		printf("RX: %c \r\n",PC_Data); 
	}
}
