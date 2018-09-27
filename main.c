/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V3.2.1
* Date               : 04/18/2011
* Description        : 
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/*
Predefine 설정 
100p
STM32F103VCT6(512KB, 64KB) :  USE_STDPERIPH_DRIVER, STM32F10X_HD
STM32F105VCT6(512KB, 64KB) :  USE_STDPERIPH_DRIVER, STM32F10X_CL
STM32F107VBT6(256KB, 64KB) :  USE_STDPERIPH_DRIVER, STM32F10X_CL
STM32F107VCT6(256KB, 64KB) :  USE_STDPERIPH_DRIVER, STM32F10X_CL
STM32F107VBT6(128KB, 32KB) :  USE_STDPERIPH_DRIVER, STM32F10X_CL

144p
STM32F103ZET6(512KB, 64KB) :  USE_STDPERIPH_DRIVER, STM32F10X_HD
*/

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "stm32f10x_tim.h"

#include "stm32f10x.h"
#include "hw_config.h"
#include "main.h"
#include "led.h"
#include "timer.h"
#include "usart.h"
#include "rtc.h"
#include "mmc_sd.h"

#include "key.h"
#include "nrf24l01.h"
#include "vs1003b.h"
#include "diskio.h"
#include "spi.h"
#include "at24c02.h"
#include "rs485.h"
#include "can.h"
#include "adc.h"
#include "dac.h"
#include "buzzer.h"

#define GPIO_Pin_PIN_ARRAY_BOTTOM        ((uint16_t)0xFF80)  /*!< All LRA pins(1~9) selected */
#define GPIO_Pin_PIN_ARRAY_TOP_1         ((uint16_t)0xFF00)  /*!< All PIN_ARRAY pins(1~8) selected */
#define GPIO_Pin_PIN_ARRAY_TOP_2         ((uint16_t)0x8000)  /*!< All PIN_ARRAY pins(9) selected */

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

int imsi = 0;
int front = 0;

// PC -> Controller 받을 데이터 구조체
typedef struct
{
        int Start_Bit;     
        int Com_Bit;
        int Pin1_Bit;
        int Pin2_Bit;
        int Time_Bit;
        int Fre_Bit;
        int Du_Bit;
        int Stop_Bit;
}RECEIVE_InitTypeDef;

// 각각의 액츄에이터 시간을 측정할 변수 구조체
typedef struct
{
        int LRA_Time;     
        int JOYSTICK_MR_Time;
        int PIN_ARRAY_Time;
        int MOVE_LRA_Time;
}Time_Measure_InitTypeDef;

// 각각의 엑츄에이터 시간측정기능을 on/off 시키는 변수 구조체
typedef struct
{
        bool LRA_Time_Flag;     
        bool JOYSTICK_MR_Time_Flag;
        bool PIN_ARRAY_Time_Flag;
        bool MOVE_LRA_Time_Flag;
}Time_Flag_InitTypeDef;


typedef struct
{
        int LRA_Time_Limit;
        int JOYSTICK_MR_Time_Limit;
        int PIN_ARRAY_Time_Limit ;
        int MOVE_LRA_Time_Limit;
} Time_Limit_InitTypeDef;



// 시관 관련 구조체 (라이브러리 함수)
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
NVIC_InitTypeDef NVIC_InitStructure;

// GPIO, ADC 구조체 (라이브러리 함수)
GPIO_InitTypeDef myGPIO_ADC;
ADC_InitTypeDef myADC;

//새로 선언한 구조체(받는 데이터, 시간 측정, 시간 적용 구조체)
RECEIVE_InitTypeDef myRecvData;
Time_Measure_InitTypeDef myTimeMeasure;
Time_Flag_InitTypeDef myTimeFlag;
Time_Limit_InitTypeDef myTimeLimit;
TIM_OCInitTypeDef  TIM_OCInitStructure;



void ALL_Actuator_Time_measurement(void);       // 시간 측정 함수
void ALL_Actuator_Set(void);
void Reset_Move_LRA();
void Down_LRA();
void Up_LRA();
void End_Move();
void changeTimerXFreq5( int freq);
void changeTimerXFreq6( int freq);
void Reset_Move_LRA_Down();
void Reset_Move_LRA_Up();
void SetPWM(double value);
void RCC_Configuration(void);

bool LRA_Toggle = FALSE;        // LRA의 전류를 교류신호처럼  주기 위한 변수
bool LRA_Toggle_Down = FALSE;        // LRA의 전류를 교류신호처럼  주기 위한 변수
bool LRA_Toggle_Up = FALSE;        // LRA의 전류를 교류신호처럼  주기 위한 변수
bool PIN_ARRAY_Toggle = FALSE;  // 핀 어레이의 전류를 교류신호처럼 주기 위한 변수
int joystick_x;                 // 조이스틱 x 값
int joystick_y;                 // 조이스틱 y 값
int send_buffer[5];            // Controller->PC로 보낼 데이터 버퍼
int recv_buffer[8];            // PC->Controller로 받을 데이터 버퍼
int Parsing_send_i=0;           // Contorller->PC 데이터 보낼 순서 변수
int Parsing_recv_i=0;           // PC->Controller 데이터 받을 순서 변수
int Move_Vibration_time = 0;

bool enUp = FALSE;
bool enDown = FALSE;  // back

bool Upen = FALSE;
bool Downen = FALSE;


void Init_Time_Limit(void){
        
        myTimeLimit.LRA_Time_Limit = 0;
        myTimeLimit.JOYSTICK_MR_Time_Limit = 0;
        myTimeLimit.PIN_ARRAY_Time_Limit = 0;
        myTimeLimit.MOVE_LRA_Time_Limit = 0;
}


// PC -> Controller 받을 데이터 구조체 초기화
void Init_RecvData(){
        
        myRecvData.Start_Bit = 0;
        myRecvData.Com_Bit = 0;
        myRecvData.Pin1_Bit = 0;
        myRecvData.Pin2_Bit = 0;
        myRecvData.Time_Bit = 0;
        myRecvData.Fre_Bit = 0;
        myRecvData.Du_Bit = 0;  
        myRecvData.Stop_Bit = 0;
        
}

// 각각의 액츄에이터 시간을 측정할 변수 구조체 초기화
void Init_Time_Measure(){
        
        myTimeMeasure.LRA_Time = 0;
        myTimeMeasure.JOYSTICK_MR_Time = 0;
        myTimeMeasure.PIN_ARRAY_Time = 0;
        myTimeMeasure.MOVE_LRA_Time = 0;
        
}

// 각각의 엑츄에이터 시간측정기능을 on/off 시키는 변수 구조체 초기화
void Init_Time_Flag(){
        
        myTimeFlag.LRA_Time_Flag = FALSE;
        myTimeFlag.JOYSTICK_MR_Time_Flag = FALSE;
        myTimeFlag.PIN_ARRAY_Time_Flag = FALSE;
        myTimeFlag.MOVE_LRA_Time_Flag = FALSE;
        
}

// LRA 전체 작동 함수(매개변수 = 작동 시간)
void LRA_ALL(int time_limit){
        
        if(myTimeMeasure.LRA_Time < time_limit*5000 && myTimeFlag.LRA_Time_Flag == TRUE){
                
                LRA_Toggle = !LRA_Toggle;
                
                if (LRA_Toggle)
                {
                        GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_SET);//turn LRA on 8
                        GPIO_WriteBit(GPIOD, GPIO_Pin_1, Bit_SET);//turn LRA on 7
                        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);//turn LRA on 6
                        GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);//turn LRA on 5
                        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET);//turn LRA on 4
                        GPIO_WriteBit(GPIOD, GPIO_Pin_5, Bit_SET);//turn LRA on 3
                        GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_SET);//turn LRA on 2
                        GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_SET);//turn LRA on 1
                        GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_SET);//turn LRA on 9
                }
                else
                {
                        GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);//turn LRA off 8
                        GPIO_WriteBit(GPIOD, GPIO_Pin_1, Bit_RESET);//turn LRA off 7
                        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);//turn LRA off 6
                        GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);//turn LRA off 5
                        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);//turn LRA off 4
                        GPIO_WriteBit(GPIOD, GPIO_Pin_5, Bit_RESET);//turn LRA off 3
                        GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_RESET);//turn LRA off 2
                        GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_RESET);//turn LRA off 1
                        GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);//turn LRA off 9
                }
        }
        else if(myTimeMeasure.LRA_Time >= time_limit*5000){
                myTimeFlag.LRA_Time_Flag =FALSE;
                myTimeLimit.LRA_Time_Limit = 0;
                myTimeMeasure.LRA_Time=0;
                GPIO_WriteBit(GPIOD, GPIO_Pin_0, Bit_RESET);//turn LRA off 8
                GPIO_WriteBit(GPIOD, GPIO_Pin_1, Bit_RESET);//turn LRA off 7
                GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);//turn LRA off 6
                GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);//turn LRA off 5
                GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);//turn LRA off 4
                GPIO_WriteBit(GPIOD, GPIO_Pin_5, Bit_RESET);//turn LRA off 3
                GPIO_WriteBit(GPIOD, GPIO_Pin_6, Bit_RESET);//turn LRA off 2
                GPIO_WriteBit(GPIOD, GPIO_Pin_7, Bit_RESET);//turn LRA off 1
                GPIO_WriteBit(GPIOC, GPIO_Pin_12, Bit_RESET);//turn LRA off 9
                
        }
        
}



void Reset_Move_LRA_Down()
{
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);//turn LRA off 3
}

void Reset_Move_LRA_Up()
{
        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);//turn LRA off 5
}

void Down_LRA(){
        
        LRA_Toggle_Down = !LRA_Toggle_Down;
        
        if (LRA_Toggle_Down)
        {
                GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);//turn LRA on 3
                
        }
        else
        {
                GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);//turn LRA off 3
        }
}

void Up_LRA(){
        LRA_Toggle_Up = !LRA_Toggle_Up;
        
        if (LRA_Toggle_Up)
        {
                GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET);//turn LRA on 5
        }
        else
        {
                GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);//turn LRA off 5
        }
}

// PIN_ARRAY 전체 작동 함수(매개변수 = 작동 시간)
void PIN_ARRAY_ALL(int time_limit){
        
        if(myTimeMeasure.PIN_ARRAY_Time < time_limit*10000 && myTimeFlag.PIN_ARRAY_Time_Flag == TRUE){
                if (PIN_ARRAY_Toggle)
                {
                        GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);//turn PIN_ARRAY on 1
                        GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET);//turn PIN_ARRAY on 2
                        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_SET);//turn PIN_ARRAY on 3
                        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_SET);//turn PIN_ARRAY on 4
                        GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_SET);//turn PIN_ARRAY on 5
                        GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_SET);//turn PIN_ARRAY on 6
                        GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_SET);//turn PIN_ARRAY on 7
                        GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_SET);//turn PIN_ARRAY on 8
                        GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET);//turn PIN_ARRAY on 9
                        
                        
                        GPIO_WriteBit(GPIOE, GPIO_Pin_7, Bit_RESET);//turn PIN_ARRAY off 1
                        GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 2
                        GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 3
                        GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 4
                        GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
                        GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 6
                        GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 7
                        GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 8
                        GPIO_WriteBit(GPIOE, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9
                        
                }
                else
                {
                        GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 1
                        GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 2
                        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 3
                        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 4
                        GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
                        GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 6
                        GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 7
                        GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 8
                        GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9
                        
                        
                        GPIO_WriteBit(GPIOE, GPIO_Pin_7, Bit_SET);//turn PIN_ARRAY on 1
                        GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_SET);//turn PIN_ARRAY on 2
                        GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_SET);//turn PIN_ARRAY on 3
                        GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_SET);//turn PIN_ARRAY on 4
                        GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_SET);//turn PIN_ARRAY on 5
                        GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_SET);//turn PIN_ARRAY on 6
                        GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_SET);//turn PIN_ARRAY on 7
                        GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_SET);//turn PIN_ARRAY on 8
                        GPIO_WriteBit(GPIOE, GPIO_Pin_15, Bit_SET);//turn PIN_ARRAY on 9
                }
        }
        else if(myTimeMeasure.PIN_ARRAY_Time*10000 >= time_limit){
                myTimeFlag.PIN_ARRAY_Time_Flag =FALSE;
                myTimeLimit.PIN_ARRAY_Time_Limit = 0;
                myTimeMeasure.PIN_ARRAY_Time=0;
                GPIO_WriteBit(GPIOE, GPIO_Pin_7, Bit_RESET);//turn PIN_ARRAY off 1
                GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 2
                GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 3
                GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 4
                GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
                GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 6
                GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 7
                GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 8
                GPIO_WriteBit(GPIOE, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9
                
                GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 1
                GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 2
                GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 3
                GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 4
                GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
                GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 6
                GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 7
                GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 8
                GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9
                
        }

}


void Init_PWM_MR(void)
{
        GPIO_InitTypeDef myGPIO;
        // TIM3 PWM channel1,2
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        myGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_50MHz;
        
        GPIO_Init(GPIOA, &myGPIO);
}

//Configure other I/O pins
void Init_GPIO_LRA_1_2_3_4_5_6_7_8(void)
{
        GPIO_InitTypeDef myGPIO;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        
        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_LRA;
        myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOD, &myGPIO);
}

void Init_GPIO_LRA_9(void)
{
        GPIO_InitTypeDef myGPIO;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        
        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_12;
        myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOC, &myGPIO);
}

void Init_GPIO_PIN_ARRAY_TOP_1(void)
{
        GPIO_InitTypeDef myGPIO;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        
        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_PIN_ARRAY_TOP_1;
        myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOD, &myGPIO);
}

void Init_GPIO_PIN_ARRAY_TOP_2(void)
{
        GPIO_InitTypeDef myGPIO;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
        
        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_PIN_ARRAY_TOP_2;
        myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOB, &myGPIO);
}

void Init_GPIO_PIN_ARRAY_BOTTOM(void)
{
        GPIO_InitTypeDef myGPIO;
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
        
        GPIO_StructInit(&myGPIO);
        myGPIO.GPIO_Pin = GPIO_Pin_PIN_ARRAY_BOTTOM;
        myGPIO.GPIO_Mode = GPIO_Mode_Out_PP;
        myGPIO.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_Init(GPIOE, &myGPIO);
}

//Configure ADC
void adc_config()
{ 
        // PA6를 analog input 모드로 설정한다.  
        myGPIO_ADC.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //set to pin 0,1
        myGPIO_ADC.GPIO_Mode = GPIO_Mode_AIN; //set as analog input
        GPIO_Init(GPIOC, &myGPIO_ADC); //set to C0,C1
        
        RCC_ADCCLKConfig(RCC_PCLK2_Div6); //clock for ADC (max 14MHz, 72/6=12MHz)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //enable ADC clock
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //enable ADC clock
        
        //configure ADC parameters
        myADC.ADC_Mode = ADC_Mode_Independent;
        myADC.ADC_ScanConvMode = DISABLE;
        myADC.ADC_ContinuousConvMode = ENABLE;
        myADC.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
        myADC.ADC_DataAlign = ADC_DataAlign_Right;
        myADC.ADC_NbrOfChannel  = 1;
        ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5); //PC0 as Input
        ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_55Cycles5); //PC1 as Input
        ADC_Init(ADC1, &myADC);
        ADC_Init(ADC2, &myADC);
        
        //enable
        ADC_Cmd(ADC1, ENABLE);
        ADC_Cmd(ADC2, ENABLE);
        
        //Calibrate ADC *optional?
        ADC_ResetCalibration(ADC1);
        while(ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);
        while(ADC_GetCalibrationStatus(ADC1));
        
        
        //Calibrate ADC *optional?
        ADC_ResetCalibration(ADC2);
        while(ADC_GetResetCalibrationStatus(ADC2));
        ADC_StartCalibration(ADC2);
        while(ADC_GetCalibrationStatus(ADC2));
        
        //enable ADC to work
        ADC_Cmd(ADC1, ENABLE);
        ADC_Cmd(ADC2, ENABLE);
}

//get Analog Value at pin
int getPot(void)
{
        return ADC_GetConversionValue(ADC1);
}

//get Analog Value at pin
int getPot2(void)
{
        return ADC_GetConversionValue(ADC2);
}

//PIN_ARRAY_Timer
void Init_TIM2(){
        NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        
        TIM_TimeBaseStructure.TIM_Prescaler = 60000-1; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 60;                  // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        ; // 1000 millisecond 가 되면 인터럽트를 발생하게 한다.
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        
        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
        
        
        TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);	
        TIM_Cmd(TIM2, ENABLE);
}

//LRA_Timer
void Init_PWM_TIM3(){
        TIM_TimeBaseStructure.TIM_Prescaler = 1440 - 1;  // 24MHz / 24 -> 1MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 500; // 50 Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // 의미 없음.

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	

	
	// PWM1 Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; // duty cycle 7.5%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// pwm channel 1
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);	
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
}

//BlueTooth_Timer
void Init_TIM4(){
        NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        
        TIM_TimeBaseStructure.TIM_Prescaler = 600-1; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 12;                 // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (1200=1Hz)
        ; // 1000 millisecond 가 되면 인터럽트를 발생하게 한다.
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        
        TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
        
        
        TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	
        TIM_Cmd(TIM4, ENABLE);
}

//LRA_Timer1
void Init_TIM5(){
        NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        
        TIM_TimeBaseStructure.TIM_Prescaler = 1440; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 110; // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        
        TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	
        
        
        TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);	
        TIM_Cmd(TIM5, ENABLE);
}

//LRA_Timer2
void Init_TIM6(){
        NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        
        TIM_TimeBaseStructure.TIM_Prescaler = 1440; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 110; // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        
        TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);	
        
        
        TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);	
        TIM_Cmd(TIM6, ENABLE);
}

void Init_TIM7(){
        NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
        
        
        TIM_TimeBaseStructure.TIM_Prescaler = 6000; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 12; // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        
        TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);	
        
        TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);	
        TIM_Cmd(TIM7, ENABLE);
}



void changeTimerXFreq5( int freq){//타이머 주기 실시간 변화
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        //SystemCoreClockUpdate();
        TIM_TimeBaseStructure.TIM_Prescaler = 1440; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 50000/(freq*2); // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
        //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
        //TIM_Cmd(TIM3, ENABLE);
}

void changeTimerXFreq6( int freq){//타이머 주기 실시간 변화
        //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
        //SystemCoreClockUpdate();
        TIM_TimeBaseStructure.TIM_Prescaler = 1440; // 1 milli second 마다 clock 이 발생한다. (24MHz / 24K) 
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = 50000/(freq*2); // 1200 = 1Hz, on/off가 적용이되면 600=1Hz (600=1Hz)
        TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 5; // 의미 없음.
        TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
        //TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
        //TIM_Cmd(TIM3, ENABLE);
        
}

void GPIO_config(){
        
        //직접 선언한 구조체 초기화
        Init_RecvData();
        Init_Time_Measure();
        Init_Time_Flag();
        Init_Time_Limit();
        
        
        Init_GPIO_LRA_1_2_3_4_5_6_7_8();
        Init_GPIO_LRA_9();
        Init_GPIO_PIN_ARRAY_BOTTOM();
        Init_GPIO_PIN_ARRAY_TOP_1();
        Init_GPIO_PIN_ARRAY_TOP_2();
        
}

int main()
{       
        
        GPIO_config(); //configure GPIO
        
        
        bsp_init_rcc();         // 마프 기본설정 함수
        RCC_Configuration();

        
        bsp_init_interrupt();      // UART 인터럽트 함수
        bsp_init_irq_usart1();     // UART 기본 설정 함수

        adc_config(); //configure ADC
        Init_PWM_MR();      // MR GPIO
        Init_PWM_TIM3();           //  MR Timer3
        Init_TIM5();            // LRA Timer2
        Init_TIM2();           // PIN_ARRAY_Timer
        Init_TIM4();           // BlueTooth_TImer
        Init_TIM6();            //Move_LRA_Timer AND LRA_Timer
        Init_TIM7();            //Move_LRA_Timer
        SetPWM(0);
        
        GPIO_WriteBit(GPIOE, GPIO_Pin_7, Bit_RESET);//turn PIN_ARRAY off 1
        GPIO_WriteBit(GPIOE, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 2
        GPIO_WriteBit(GPIOE, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 3
        GPIO_WriteBit(GPIOE, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 4
        GPIO_WriteBit(GPIOE, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
        GPIO_WriteBit(GPIOE, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 6
        GPIO_WriteBit(GPIOE, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 7
        GPIO_WriteBit(GPIOE, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 8
        GPIO_WriteBit(GPIOE, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9

        GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 1
        GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_RESET);//turn PIN_ARRAY off 2
        GPIO_WriteBit(GPIOD, GPIO_Pin_13, Bit_RESET);//turn PIN_ARRAY off 3
        GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);//turn PIN_ARRAY off 4
        GPIO_WriteBit(GPIOD, GPIO_Pin_11, Bit_RESET);//turn PIN_ARRAY off 5
        GPIO_WriteBit(GPIOD, GPIO_Pin_10, Bit_RESET);//turn PIN_ARRAY off 6
        GPIO_WriteBit(GPIOD, GPIO_Pin_9, Bit_RESET);//turn PIN_ARRAY off 7
        GPIO_WriteBit(GPIOD, GPIO_Pin_8, Bit_RESET);//turn PIN_ARRAY off 8
        GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET);//turn PIN_ARRAY off 9
        //usart1_transmit_string("AT+COMMAND");
        //usart1_transmit_string("AT+SERVER=P");
        usart1_transmit_string("AT+UART=115200\r");
        usart1_transmit_string("AT+BYPASS\r");
        while (1)
        {
                if(myTimeFlag.JOYSTICK_MR_Time_Flag == TRUE)
                {
                        if(imsi == 1)
                        {
                                SetPWM(139);
                                myTimeFlag.JOYSTICK_MR_Time_Flag = FALSE;
                        }
                        else if(imsi == 2)
                        {
                                SetPWM(0);
                                myTimeFlag.JOYSTICK_MR_Time_Flag = FALSE;
                        }
                }
//                  if(USART1->SR &  USART_FLAG_TXE)
//                       USART1->DR = (uint16_t)'5';
//                 if(USART1->SR &  USART_FLAG_TXE)
//                       USART1->DR = (uint16_t)'\r';
              //  while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE)); 
                //USART_SendData(USART1, 'A');
         
        }
}

//PIN_ARRAY_Timer_Handler (Command = 3)
void TIM2_IRQHandler(void)
{
        // TIM2 이벤트 호출.
        
        if(TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET)
        {
                
                if(myTimeFlag.PIN_ARRAY_Time_Flag == TRUE){       
                        PIN_ARRAY_Toggle = !PIN_ARRAY_Toggle;
                        PIN_ARRAY_ALL(1 );       
                }
                
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // Clear the interrupt flag
        }	
}

/*
//MR_JOYSTICK_Timer_Handler (Receive)
void TIM3_IRQHandler(void){
        if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
        {
                TIM_ClearITPendingBit(TIM3, TIM_IT_Update);          

                
        }
}*/


//BlueTooth_Timer_Handler (Receive)
void TIM4_IRQHandler(void)
{
        // TIM2 이벤트 호출.
        
        if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)
        {
                joystick_x = (int)(getPot()/16.33);             // 조이스틱 x 값 받기
                joystick_y = (int)(getPot2()/16.33);            // 조이스틱 y 값 받기
                
                
                
               send_buffer[0]=0xFA;                 // start bit
                //send_buffer[1]=0;                    // button input(0~6)
                //send_buffer[2]=0;                    // button input(0~6)
                send_buffer[1]=joystick_x;           // joystick_x value
                send_buffer[2]=joystick_y;           // joystick_y value
                send_buffer[3]=0xFB;               // stop bit
                
                //USART_SendData(USART1,send_buffer[Parsing_send_i]); 
                USART_SendData(USART1,send_buffer[Parsing_send_i]); 
                USART_SendData(USART1,'\r');
                
                // USART_SendData(USART1,(uint8_t)'5'); 
                //USART_SendData(USART1,'\r');

                //USART_SendData(USART1,'\n');
                //usart1_transmit_string_format("bluetooth test, Parsing: %d, send_value: %d \r\n",Parsing_send_i,send_buffer[Parsing_send_i]);
                //usart1_transmit_string_format("bluetooth test, x: %d, y: %d \r\n",joystick_x,joystick_y); 
               // USART_SendData(port, chr);
               
                if(Parsing_send_i==3){
                        Parsing_send_i=-1;
                }
                
                 Parsing_send_i++;
                
                
                Move_Vibration_time++;
                ALL_Actuator_Time_measurement(); // 시간 측정 함수
                
                TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // Clear the interrupt flag
        }	
}

//LRA_Timer_Handler (Command = 1) Move_LRA_Timer_Down(Command = 4)
void TIM5_IRQHandler(void)
{
        // TIM5 이벤트 호출.
        
        //Down_LRA();
        
        if(TIM_GetITStatus(TIM5,TIM_IT_Update) != RESET)
        {     
                  
                
                if(enDown)
                        Down_LRA();
                if(Downen)
                        Up_LRA();
                

                if(myTimeFlag.LRA_Time_Flag == TRUE)
                {
                       // LRA_ALL(myTimeLimit.LRA_Time_Limit);
                         LRA_ALL(1);
                }
                TIM_ClearITPendingBit(TIM5, TIM_IT_Update); // Clear the interrupt flag
                
        }	
}

        
//Move_LRA_Timer_Up(Command = 4)
void TIM6_IRQHandler(void)
{
        // TIM6 이벤트 호출.
        
        if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
        {    
                if(enUp)
                        Up_LRA();
                if(Upen)
                        Down_LRA();
                TIM_ClearITPendingBit(TIM6, TIM_IT_Update); // Clear the interrupt flag
        }	
}

float moving_obj = 0;           //가상 물체의 위치
int STEP = 4; //mm
int prev_pos = 0;
bool enBurst = FALSE;
int burst_time = 23;            //ms
int Device_length = 150; //mm
bool enTVW = FALSE;
int Up_Hz = 240;
int Down_Hz = 240;

void TIM7_IRQHandler(void)
{
        // TIM7 이벤트 호출.
        static float v = 250 ;//mm/s
        //static float acc = 100 ;//mm/s^2
        static int burst_cnt = 0;
        
        if(TIM_GetITStatus(TIM7,TIM_IT_Update) != RESET)
        {     
                if(front == 1)
                {
                        if(enTVW){
                                moving_obj += v/1000.0; 
                                //                        v += acc / 1000.0;
                                if(moving_obj >=Device_length){
                                        enTVW = FALSE; 
                                        moving_obj = 0;
                                        prev_pos = 0; 
                                        enBurst = FALSE;
                                        burst_cnt = 0 ;
                                        enDown = FALSE;
                                        enUp = FALSE;
                                        v = 250;
                                        //acc = 100;
                                        changeTimerXFreq5( Up_Hz);
                                        changeTimerXFreq6( Up_Hz);
                                        End_Move();
                                        front = 0;
                                        return;
                                }
                                if((moving_obj - prev_pos) > STEP) {
                                        enBurst = TRUE;
                                        prev_pos = moving_obj;
                                }       
                                if(enBurst){
                                        burst_cnt++;
                                        if(burst_cnt < burst_time){
                                                if(moving_obj < Device_length/3 ){
                                                        enUp = TRUE;
                                                        enDown= FALSE;     
                                                }else if(moving_obj < Device_length * 2/3){
                                                        enUp = TRUE;
                                                        enDown = TRUE;  
                                                }else{
                                                        enUp = FALSE;
                                                        enDown = TRUE;  
                                                }                            
                                        }else{
                                                enBurst =FALSE;
                                                burst_cnt = 0 ;
                                                enDown = FALSE;
                                                enUp = FALSE;
                                        }
                                }
                        }
                }
                else if(front == 2)
                {
                        if(enTVW){
                                moving_obj += v/1000.0; 
                                //                        v += acc / 1000.0;
                                if(moving_obj >=Device_length){
                                        enTVW = FALSE; 
                                        moving_obj = 0;
                                        prev_pos = 0; 
                                        enBurst = FALSE;
                                        burst_cnt = 0 ;
                                        Downen = FALSE;
                                        Upen = FALSE;
                                        v = 250;
                                        //acc = 100;
                                        changeTimerXFreq5( Up_Hz);
                                        changeTimerXFreq6( Up_Hz);
                                        End_Move();
                                        front = 0;
                                        return;
                                }
                                if((moving_obj - prev_pos) > STEP) {
                                        enBurst = TRUE;
                                        prev_pos = moving_obj;
                                }       
                                if(enBurst){
                                        burst_cnt++;
                                        if(burst_cnt < burst_time){
                                                if(moving_obj < Device_length/3 ){
                                                        Upen = TRUE;
                                                        Downen= FALSE;     
                                                }else if(moving_obj < Device_length * 2/3){
                                                        Upen = TRUE;
                                                        Downen = TRUE;  
                                                }else{
                                                        Upen = FALSE;
                                                        Downen = TRUE;  
                                                }                            
                                        }else{
                                                enBurst =FALSE;
                                                burst_cnt = 0 ;
                                                Downen = FALSE;
                                                Upen = FALSE;
                                        }
                                }
                        }
                }       
                TIM_ClearITPendingBit(TIM7, TIM_IT_Update); // Clear the interrupt flag
        }	
}

void USART1_IRQHandler(void)
{
        // PC -> Controller 값 받기 (블루투스 통신)
        if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        {
                recv_buffer[Parsing_recv_i] =  USART_ReceiveData(USART1);
                
                
              
                if(Parsing_recv_i == 0){
                        myRecvData.Start_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 1){
                        myRecvData.Com_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 2){
                        myRecvData.Pin1_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 3){
                        myRecvData.Pin2_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 4){
                        myRecvData.Time_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 5){
                        myRecvData.Fre_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 6){
                        myRecvData.Du_Bit=recv_buffer[Parsing_recv_i];
                }
                else if(Parsing_recv_i == 7){
                        myRecvData.Stop_Bit=recv_buffer[Parsing_recv_i];
                }
                
               // usart1_transmit_string_format("%c",recv_buffer[Parsing_recv_i] );
                
                if(myRecvData.Start_Bit == 250){
                        Parsing_recv_i++;
                }
                
                if(Parsing_recv_i==8 && myRecvData.Stop_Bit != 251)
                {
                        Init_RecvData();
                        Parsing_recv_i = 0;
                }
                
                
                if(Parsing_recv_i==8 && myRecvData.Stop_Bit ==251 ){
                        ALL_Actuator_Set();
                        imsi = myRecvData.Pin2_Bit;
                        front  = myRecvData.Du_Bit;
                        Init_RecvData();
                        Parsing_recv_i=0;
                }
                
                
                USART_ClearITPendingBit(USART1, USART_IT_RXNE);	  
        }
}

void End_Move()
{
        myTimeFlag.MOVE_LRA_Time_Flag = FALSE;
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);//turn LRA off 3
        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);//turn LRA off 3
        Move_Vibration_time = 0;
}

void ALL_Actuator_Set(void){
        // 어떤걸 시간 측정할 것인지 선택
        if(myRecvData.Start_Bit == 250 && myRecvData.Stop_Bit == 251){
                if(myRecvData.Com_Bit == 1){
                        myTimeMeasure.LRA_Time = 0;
                        myTimeLimit.LRA_Time_Limit = myRecvData.Time_Bit;
                        myTimeFlag.LRA_Time_Flag = TRUE;                 
                }
                else if(myRecvData.Com_Bit == 2){
                        myTimeMeasure.JOYSTICK_MR_Time = 0;
                        myTimeLimit.JOYSTICK_MR_Time_Limit = myRecvData.Time_Bit;
                        myTimeFlag.JOYSTICK_MR_Time_Flag = TRUE;       
                }
                else if(myRecvData.Com_Bit == 3){
                        myTimeMeasure.PIN_ARRAY_Time = 0;
                        myTimeLimit.PIN_ARRAY_Time_Limit = myRecvData.Time_Bit;
                        myTimeFlag.PIN_ARRAY_Time_Flag = TRUE;                 
                }
                else if(myRecvData.Com_Bit == 4){
                        myTimeMeasure.MOVE_LRA_Time = 0;
                        myTimeLimit.MOVE_LRA_Time_Limit = myRecvData.Time_Bit;
                        Move_Vibration_time = 0;
                        enTVW = TRUE;
                }          
        }
}

int flag = 0;

// 모든 액츄에이터 시간 측정 함수
void ALL_Actuator_Time_measurement(void){
        
        // 시간측정
        if(myTimeFlag.LRA_Time_Flag == TRUE){
                myTimeMeasure.LRA_Time++;
        }
        else if(myTimeFlag.LRA_Time_Flag == FALSE){
                myTimeMeasure.LRA_Time = 0;
        }
        if(myTimeFlag.JOYSTICK_MR_Time_Flag == TRUE){
                myTimeMeasure.JOYSTICK_MR_Time++;
        }
        else if(myTimeFlag.JOYSTICK_MR_Time_Flag == FALSE){
                myTimeMeasure.JOYSTICK_MR_Time = 0;
        }
        if(myTimeFlag.PIN_ARRAY_Time_Flag == TRUE){
                myTimeMeasure.PIN_ARRAY_Time++;
        }
        else if(myTimeFlag.PIN_ARRAY_Time_Flag == FALSE){
                myTimeMeasure.PIN_ARRAY_Time = 0;
        }
        if(myTimeFlag.MOVE_LRA_Time_Flag == TRUE){
                myTimeMeasure.MOVE_LRA_Time++;
        }
        else if(myTimeFlag.MOVE_LRA_Time_Flag == FALSE){
                myTimeMeasure.MOVE_LRA_Time = 0;
        }
        
}

void SetPWM(double value)
{       	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = value;
	// pwm channel 1
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);	
}


void RCC_Configuration(void)
{
        /* TIM1, GPIOA and GPIOB clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);
        
        /* TIM2 and TIM3 and TIM4 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5  |  RCC_APB1Periph_TIM6  |  RCC_APB1Periph_TIM7 , ENABLE);
}

/**
* @brief  Configures the different system clocks.
* @param  None
* @retval None
*/



