/**
******************************************************************************
* @author   2023-10-31 by Hanmin
* @brief   Embedded Controller:  LAB - USART
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSysTick.h"


static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;
void LED_toggle();

void setup(void){
    RCC_PLL_init();

    // USB serial init
    UART2_init();
    UART2_baud(BAUD_38400);

    // BT serial init
    UART1_init();
    UART1_baud(BAUD_38400);
   
      //LED setup
      GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()   

}


void main(){
    setup();

    while(1){
           
         
    }
}


void USART2_IRQHandler(){                // USART2 RX Interrupt : Recommended
    if(is_USART2_RXNE()){
        PC_Data = USART2_read();      // RX from UART2 (PC)
        USART2_write(&PC_Data,1);      // TX to USART1    (BT)
            USART1_write(&PC_Data,1);
        printf("MCU_1 sent : %c \r\n",PC_Data); // TX to USART2(PC)
    }
}


void USART1_IRQHandler(){                
    if(is_USART1_RXNE()){      
         PC_Data = USART1_read();      
         
			if(PC_Data == 'L'){
            GPIO_write(GPIOA, LED_PIN, 0);
         }
         else if(PC_Data == 'H'){
            GPIO_write(GPIOA, LED_PIN, 1);
         }
       
         
      printf("MCU_1 received : %c \r\n",PC_Data); // TX to USART2(PC)
         
    }
}
