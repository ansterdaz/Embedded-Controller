/**
******************************************************************************
* @author	Hanmin
* @Mod		2023.09.23
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

#define LED_PIN 	5
#define BUTTON_PIN 13

int buttonState = 0;
int ledState = 0;
	

void setup(void);
	
int main(void) { 
	setup();
	
	while(1){
		
	int currentButtonState = GPIO_read(GPIOC,BUTTON_PIN);
		
	if(currentButtonState == 0 && buttonState ==1) 
	{
		ledState += 1;
		
		if(ledState == 1)          //  first LED on
		{
		 GPIO_write(GPIOA,LED_PIN,HIGH);
		 GPIO_write(GPIOA,6,LOW);
		 GPIO_write(GPIOA,7,LOW);
		 GPIO_write(GPIOB,6,LOW);
		}
		else if(ledState ==2)     // second LED on
		{
		 GPIO_write(GPIOA,LED_PIN,LOW);
		 GPIO_write(GPIOA,6,HIGH);
		 GPIO_write(GPIOA,7,LOW);
		 GPIO_write(GPIOB,6,LOW);
		}
		else if(ledState ==3)     // third LED on
		{
		 GPIO_write(GPIOA,LED_PIN,LOW);
		 GPIO_write(GPIOA,6,LOW);
		 GPIO_write(GPIOA,7,HIGH);
		 GPIO_write(GPIOB,6,LOW);
		}
		else if(ledState==4)     // Forth LED on 
		{
		 GPIO_write(GPIOA,LED_PIN,LOW);
		 GPIO_write(GPIOA,6,LOW);
		 GPIO_write(GPIOA,7,LOW);
		 GPIO_write(GPIOB,6,HIGH);
		 ledState = 0;
    }
	}
	  buttonState = currentButtonState;
}	 	
} 	

// Initialiization 
void setup(void)
{
	RCC_HSI_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_init(GPIOA, 6, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_init(GPIOA, 7, OUTPUT);    // calls RCC_GPIOA_enable()	
	GPIO_init(GPIOB, 6, OUTPUT);    // calls RCC_GPIOB_enable()
	
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);                // Pull up
	GPIO_otype(GPIOA, LED_PIN, EC_PUSH_PULL);        // push pull
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);          // medium speed
	
	GPIO_pupd(GPIOA, 6, EC_PU);                      // Pull up
	GPIO_otype(GPIOA, 6, EC_PUSH_PULL);              // push pull
	GPIO_ospeed(GPIOA, 6, EC_MEDIUM);                // medium speed
	
	GPIO_pupd(GPIOA, 7, EC_PU);                      // Pull up
	GPIO_otype(GPIOA, 7, EC_PUSH_PULL);              // push pull
	GPIO_ospeed(GPIOA, 7, EC_MEDIUM);                // medium speed
	
	GPIO_pupd(GPIOB, 6, EC_PU);                      // Pull up
	GPIO_otype(GPIOB, 6, EC_PUSH_PULL);              // push pull
	GPIO_ospeed(GPIOB, 6, EC_MEDIUM);                // medium speed
	
	
}
