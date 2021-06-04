#ifndef _INCLUDE_H_
#define _INCLUDE_H_
#include "adc-control.h"
#include "sanchalukou.h"
#include "headfile.h"
#include "steer.h"
#include "motor.h"
#include "Function.h"
#include "Standing.h"
//#include "gyroRain.h"
#define POWER_ADC0_MOD  ADC_0       //定义通道一 ADC模块号
#define POWER_ADC0_Pin  ADC0_CH0_A0 //定义通道一 ADC引脚

#define POWER_ADC1_MOD  ADC_0       //定义通道一 ADC模块号
#define POWER_ADC1_Pin  ADC0_CH1_A1 //定义通道一 ADC引脚

#define POWER_ADC2_MOD  ADC_0       //定义通道一 ADC模块号
#define POWER_ADC2_Pin  ADC0_CH2_A2 //定义通道一 ADC引脚

#define POWER_ADC3_MOD  ADC_0       //定义通道一 ADC模块号
#define POWER_ADC3_Pin  ADC0_CH3_A3 //定义通道一 ADC引脚

#define POWER_ADC4_MOD  ADC_0       //定义通道一 ADC模块号
#define POWER_ADC4_Pin  ADC0_CH4_A4 //定义通道一 ADC引脚

#define S_MOTOR_PIN   ATOM1_CH1_P33_9       //定义舵机引脚

#define MOTOR_A   ATOM0_CH0_P21_2   //定义1电机正转PWM引脚
#define MOTOR_B   ATOM0_CH1_P21_3   //定义1电机反转PWM引脚



typedef struct
{
	float P;
	float I;
    float D1;//直道
    float D2;//环岛
	float POut;
	float IOut;
	float DOut;

}PID1;



typedef struct
{

        int weqrgf;


}Tags;

extern Tags Tags_rain;
extern PID1 PID_rain;
extern int sflag;
extern int tflag;
extern int    cr_flag1_l;
extern int    cr_flag1_r;                               //环岛标志
extern uint8  Flag_Round;                               //进 环 位 置 标 志 位

#endif
