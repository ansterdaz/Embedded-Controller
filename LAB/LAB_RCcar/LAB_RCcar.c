/**
******************************************************************************
* @author   2023-11-23 by Hanmin Kim
* @brief   Embedded Controller:  LAB - Line Tracing Rc Car
*
******************************************************************************
*/
#include "stm32f4xx.h"
#include "ecSTM32F411.h"
#include "math.h"
#include "stdio.h"

// DIR pin
#define DIR_PIN1 2
#define DIR_PIN2 3
//PWM pin
PinName_t PWM_PIN1 = PA_0;
PinName_t PWM_PIN2 = PA_1;
//UltraSonic pin
#define TRIG PA_6
#define ECHO PB_6
//Emergency v 
#define EX 1
// velocity 
#define v0 0.7
#define v1 0.5
#define v2 0.25
#define v3 0
#define F 1
#define B 0

uint32_t _count = 0;
float period = 6;
// UltraSonic
uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
//IR parameter//
uint32_t value1, value2;
int flag = 0;
PinName_t seqCHn[2] = {PB_0, PB_1};

static volatile uint8_t PC_Data = 0;
static volatile uint8_t BT_Data = 0;

int i=0;
char mode;
double vel[4] = {v0, v1, v2, v3};
int str_level = 0;
double vel1 = 1;
double vel2 = 1;
char DIR;
char VEL[2];
char STR[2];
uint8_t dir = 1;

void setup(void);
double str_angle(int str_level);
void printState(void);
void speedUP();
void speedDOWN();
void M_right();
void M_left();
void M_straight();
void E_stop();
void M_back();
void LED_toggle();
void main(){
    setup();
    GPIO_write(GPIOC, DIR_PIN1, dir);
    GPIO_write(GPIOC, DIR_PIN2, dir);
    PWM_duty(PWM_PIN1, vel1);
    PWM_duty(PWM_PIN2, vel2);
    while(1){
			    // Auto Mode
        if(mode == 'A'){
            distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
					  
					// Forward
            if(value1 < 1000 && value2 < 1000){      
                vel1 = 0;
                vel2 = 0;
            }
				  // right
            else if(value1 > 1000 && value2 < 1000){ // 
                vel1 = 0;
                vel2 = 0.6;
            }
					// left
            else if(value1 < 1000 && value2 > 1000){
                vel1 = 0.6;
                vel2 = 0;			
            }
				  // stop
            else if(value1 > 1000 && value2 > 1000){
                vel1 = 1;
                vel2 = 1;
            }
						// Obstacle Detection
						if(distance < 7){                        
							E_stop();					
						}
						else if(distance > 3000){
							continue;
						}
						if(_count >= 1){
							LED_toggle();
							printState();
							_count = 0;
						}
					GPIO_write(GPIOC, DIR_PIN1, dir);
					GPIO_write(GPIOC, DIR_PIN2, dir);
					PWM_duty(PWM_PIN1, vel1);
					PWM_duty(PWM_PIN2, vel2);
        }
        if(mode == 'M'){
					if((_count >= 2) &(BT_Data!='E')){
						printState();
						_count = 0;
					}
					GPIO_write(GPIOA, LED_PIN, 1);
					GPIO_write(GPIOC, DIR_PIN1, dir);
					GPIO_write(GPIOC, DIR_PIN2, dir);
					PWM_duty(PWM_PIN1, vel1);
					PWM_duty(PWM_PIN2, vel2);
        }
    }
}
void USART1_IRQHandler(){                   
    if(is_USART1_RXNE()){
        BT_Data = USART1_read();
			// Manual Mode
        if(BT_Data == 'M') {
						USART1_write("Manual Mode\r\n",13);
            mode = 'M';
        }
			// Auto Mode
        else if(BT_Data == 'A'){
						USART1_write("Auto Mode\r\n",11);
            mode = 'A';
        }
		  // Velocity increase
        if(mode == 'M') {
            if (BT_Data == '>'){
                speedUP();
            }
		  // Velocity decrease
            else if (BT_Data == '<'){
                speedDOWN();
            }
		  // right
            else if (BT_Data == 'd') {
                M_right();
		  // left
            } else if (BT_Data == 'a') {
                M_left();
            } else if (BT_Data == 'w') {
                M_straight();
            }
		  // back
            else if (BT_Data == 's') {
                M_back();
            }
		  // Emergency Stop
            else if (BT_Data == 'E'){
                E_stop();
								USART1_write("Emergency\r\n", 11);
            }
        }
    }
}

// Print the RC car state 
void printState(void){
	
	if(mode == 'M'){
		sprintf(VEL, "%d", i);
		sprintf(STR, "%d", str_level);
		
		USART1_write("MOD: ", 5);
		USART1_write(&mode, 1);
		USART1_write(" DIR: ", 6);
		USART1_write(&DIR, 1);
		USART1_write(" STR: ", 6);
		USART1_write(&STR, 2);
		USART1_write(" VEL: ", 6);
		if(str_level == 0){
		USART1_write("V", 1);
		USART1_write(&VEL, 2);
		}
		USART1_write("\r\n", 2);
	}
	
	else if(mode == 'A'){
		if(distance < 7){
			USART1_write("Obstacle Infront\r\n", 18);
		}
		else{
			if(value1 < 1000 && value2 < 1000){
				USART1_write("Straight\r\n",10);
			}
			else if(value1 > 1000 && value2 < 1000){
				USART1_write("Turn right\r\n", 13);
			}
			else if(value1 < 1000 && value2 > 1000){
				USART1_write("Turn left\r\n", 12);
			}
		}
	}
}
// IR sensor 
void ADC_IRQHandler(void){
    if(is_ADC_OVR())
        clear_ADC_OVR();

    if(is_ADC_EOC()){      // after finishing sequence
        if (flag==0)
            value1 = ADC_read();
        else if (flag==1)
            value2 = ADC_read();
        flag =! flag;      // flag toggle
    }
}
// UltraSonic
void TIM4_IRQHandler(void){
    if(is_UIF(TIM4)){                     // Update interrupt
        ovf_cnt++;													// overflow count
				_count++;
        clear_UIF(TIM4);  							    // clear update interrupt flag
    }
    if(is_CCIF(TIM4, 1)){ 								// TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
        time1 = TIM4->CCR1;									// Capture TimeStart
        clear_CCIF(TIM4, 1);                // clear capture/compare interrupt flag
    }
    else if(is_CCIF(TIM4, 2)){ 									// TIM4_Ch2 (IC2) Capture Flag. Falling Edge Detect
        time2 = TIM4->CCR2;									// Capture TimeEnd
        timeInterval = ((time2 - time1) + (TIM4->ARR+1) * ovf_cnt) * 0.01; 	// (10us * counter pulse -> [msec] unit) Total time of echo pulse
        ovf_cnt = 0;                        // overflow reset
        clear_CCIF(TIM4,2);								  // clear capture/compare interrupt flag
    }
}

void speedUP(){
    i++;
    if(i>=3) i=3;
    vel1 = vel[i];
    vel2 = vel[i];
}

void speedDOWN(){
    i--;
    if(i<=0) i=0;
    vel1 = vel[i];
    vel2 = vel[i];
}

void M_right(){
    str_level--;
		if(str_level<-3) str_level=-3;
    str_angle(str_level);
}

void M_left(){
    str_level++;
		if(str_level>3) str_level=3;
    str_angle(str_level);
}

void M_straight(){
    str_level = 0;
    dir = F;
    vel1 = vel[i];
    vel2 = vel[i];
		DIR = 'F';
}

void M_back(){
    str_level = 0;
    dir = B;
    vel1 = vel[1];
		vel2 = vel[1];
		DIR = 'B';
}

void E_stop(){
    dir = F;
    vel1 = EX;
    vel2 = EX;
}

double str_angle(int str_level){
    if(str_level == -1){
        vel1 = v2;
        vel2 = v1;
    }
    else if(str_level == -2){
        vel1 = v2;
        vel2 = v0;
    }
    else if(str_level == -3){
        vel1 = v3;
        vel2 = v0;
    }
    else if(str_level == 1){
        vel1 = v1;
        vel2 = v2;
    }
    else if(str_level == 2){
        vel1 = v0;
        vel2 = v2;
    }else if(str_level == 3){
        vel1 = v0;
        vel2 = v3;
    }
    else if(str_level == 0){
        vel1 = vel[i];
        vel2 = vel[i];
    }
}

void LED_toggle(void){
    static unsigned int out = 0;
    if(out == 0) out = 1;
    else if(out == 1) out = 0;
    GPIO_write(GPIOA, LED_PIN, out);
}

void setup(void){
    RCC_PLL_init();
    SysTick_init();                     // SysTick Init
    UART2_init();
    // LED
    GPIO_init(GPIOA,LED_PIN,OUTPUT);

    // BT serial init
    UART1_init();
    UART1_baud(BAUD_9600);

    // DIR1 SETUP
    GPIO_init(GPIOC, DIR_PIN1, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN1, EC_PUSH_PULL);

    // DIR2 SETUP
    GPIO_init(GPIOC, DIR_PIN2, OUTPUT);
    GPIO_otype(GPIOC, DIR_PIN2, EC_PUSH_PULL);

    // ADC Init
    ADC_init(PB_0);
    ADC_init(PB_1);

    // ADC channel sequence setting
    ADC_sequence(seqCHn, 2);

    // PWM1
    PWM_init(PWM_PIN1);
    PWM_period_ms(PWM_PIN1, period);

    // PWM2
    PWM_init(PWM_PIN2);
    PWM_period_ms(PWM_PIN2, period);

    // PWM configuration ---------------------------------------------------------------------
    PWM_init(TRIG);			// PA_6: Ultrasonic trig pulse
    PWM_period_us(TRIG, 50000);    // PWM of 50ms period. Use period_us()
    PWM_pulsewidth_us(TRIG, 10);   // PWM pulse width of 10us

    // Input Capture configuration -----------------------------------------------------------------------
    ICAP_init(ECHO);    	// PB_6 as input caputre
    ICAP_counter_us(ECHO, 10);   	// ICAP counter step time as 10us
    ICAP_setup(ECHO, 1, IC_RISE);  // TIM4_CH1 as IC1 , rising edge detect
    ICAP_setup(ECHO, 2, IC_FALL);  // TIM4_CH2 as IC2 , falling edge detect

}
