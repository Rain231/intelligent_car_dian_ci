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
#define POWER_ADC0_MOD  ADC_0       //����ͨ��һ ADCģ���
#define POWER_ADC0_Pin  ADC0_CH0_A0 //����ͨ��һ ADC����

#define POWER_ADC1_MOD  ADC_0       //����ͨ��һ ADCģ���
#define POWER_ADC1_Pin  ADC0_CH1_A1 //����ͨ��һ ADC����

#define POWER_ADC2_MOD  ADC_0       //����ͨ��һ ADCģ���
#define POWER_ADC2_Pin  ADC0_CH2_A2 //����ͨ��һ ADC����

#define POWER_ADC3_MOD  ADC_0       //����ͨ��һ ADCģ���
#define POWER_ADC3_Pin  ADC0_CH3_A3 //����ͨ��һ ADC����

#define POWER_ADC4_MOD  ADC_0       //����ͨ��һ ADCģ���
#define POWER_ADC4_Pin  ADC0_CH4_A4 //����ͨ��һ ADC����

#define S_MOTOR_PIN   ATOM1_CH1_P33_9       //����������

#define MOTOR_A   ATOM0_CH0_P21_2   //����1�����תPWM����
#define MOTOR_B   ATOM0_CH1_P21_3   //����1�����תPWM����



typedef struct
{
	float P;
	float I;
    float D1;//ֱ��
    float D2;//����
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
extern int    cr_flag1_r;                               //������־
extern uint8  Flag_Round;                               //�� �� λ �� �� ־ λ

#endif
