/**
******************************************************************************
* @author   2023-12-10  Hanmin Kim, Seongbin Moon
* @brief   Embedded Controller:  Final
*
******************************************************************************
*/
#include "ecSTM32F411.h"

//--Servo Motor1--//
#define PWM_PIN PA_1


//--Servo Motor2--//
#define PWM_PIN3 PB_10


//Piezo parameter//
uint32_t PI1; //Heavy detect
uint32_t PI3; //Start line detect


int flag = 0;
PinName_t seqCHn[2] = {PB_0, PB_1};




void setup(void);
   
int main(void) { 
   
   // Initialiization --------------------------------------------------------
   setup();
   
   // Inifinite Loop ----------------------------------------------------------
   while(1){
      printf("PI1 = %d \r\n",PI1); 
      printf("PI3 = %d \r\n",PI3);
      printf("\r\n");
		 
		
      delay_ms(1000);
   }
}

// Initialiization 
void setup(void)
{   
   RCC_PLL_init();                         // System Clock = 84MHz
   UART2_init();
   SysTick_init();
   
   // ADC setting
  ADC_init(PB_0); //Start line
  ADC_init(PB_1); //Heavy object detect
    
    // ADC channel sequence setting
   ADC_sequence(seqCHn, 2);
   
   // PWM of 20 msec:  TIM2_CH2 (PA_1 AFmode)
   PWM_init(PWM_PIN);
   PWM_period(PWM_PIN,20);   // 20 msec PWM period
   
   
    // PWM of 20 msec:  TIM2_CH3 (PB_10 AFmode)   
   PWM_init(PWM_PIN3);
   PWM_period(PWM_PIN3,20);   // 20 msec PWM period
  
   TIM_UI_init(TIM3, 1);         // TIM3 Update-Event Interrupt every 1 msec 
   TIM_UI_enable(TIM3);

}

void ADC_IRQHandler(void){
   if((is_ADC_OVR())){ 
       clear_ADC_OVR();
   }
   
   if(is_ADC_EOC()){       //after finishing sequence

         if(flag==0){ 
             PI1 = ADC_read();

          }
         else if(flag==1){ 
             PI3 = ADC_read();
            
          }
          flag =!flag;
   }
}


void TIM3_IRQHandler(void){ 
   
   if(is_UIF(TIM3)){         // Check UIF(update interrupt flag)
		 
       if (PI1 >1000.0) { //Pressure sensor detected on
           PWM_duty(PWM_PIN,(1.5/20));  //servo motor rotate 90 degree
          }
       else if (PI1 <1000.0){
           PWM_duty(PWM_PIN,(0.5/20));  //servo motor comes back 0 degree
       }


       if (PI3 >1000.0) {  //Pressure sensor detected on
           PWM_duty(PWM_PIN3,(1.5/20)); //servo motor rotate 90 degree 
          }
       else if (PI3 <1000.0){
           PWM_duty(PWM_PIN3,(0.5/20)); //servo motor comes back 0 degree
       }

    
      clear_UIF(TIM3);       // Clear UI flag by writing 0
   }
}