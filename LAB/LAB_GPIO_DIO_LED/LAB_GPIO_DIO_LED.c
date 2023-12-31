/**
******************************************************************************
* @author	Hanmin
* @Mod		2023.09.23
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED by Button B1 pressing
* 
******************************************************************************
*/


#define LED_PIN 	5
#define BUTTON_PIN 13

int buttonState = 0;
int ledState = 0;

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

void setup(void);
	
int main(void) { 
	// Initialiization
	setup();
	
	// Inifinite Loop 
	while(1){
		
	int currentButtonState = GPIO_read(GPIOC, BUTTON_PIN);

        if (currentButtonState == 0 && buttonState == 1) 
				{
           ledState = !ledState; 

						if (ledState == 1)
							{
                GPIO_write(GPIOA, LED_PIN, HIGH);
              } 
						else 
							{
                GPIO_write(GPIOA, LED_PIN, LOW);
              }
        }    
        buttonState = currentButtonState;
    }

    return 0;
	
	}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_pupd(GPIOA, LED_PIN, EC_PU);
	GPIO_otype(GPIOA, LED_PIN, 0);
	
        GPIO_pupd(GPIOA, LED_PIN, EC_PU);     // Pull up
	GPIO_otype(GPIOA, LED_PIN, EC_OPEN_DRAIN);        // Open- Drain
	GPIO_ospeed(GPIOA, LED_PIN, EC_MEDIUM);           // medium speed
}
