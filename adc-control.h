/*
 * adc-control.h
 *
 *  Created on: 2021��3��3��
 *      Author: dade
 */
#ifndef CODE_ADC_CONTROL_H_
#define CODE_ADC_CONTROL_H_
#include "include.h"


/*********������ƺ���********************/
extern int AD_VAL4;//����ADֵ
extern void Get_AD(void);
extern float  ad[5];
extern float DirectionError[3]; //��DirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ�
extern float DirectionError_dot[2];   //��DirectionError_dot[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��΢�֣�
//��DirectionError_dot[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ��΢�֣�
extern void shangweiji(void);
extern float ad0;
extern float ad1;
extern int    L_huan;
extern int    R_huan;


extern float temp_1;
extern float temp_2;
extern float temp_3;
extern float temp_4;
extern int    In_Flag_Right;
extern int    cr_flag1;

extern int oflag;
/************ips��ʾ����*********/
extern float abs_f(float k);

extern void ips_adcprint(void);

extern float Cha_BI_He(float Date_1,float Date_2,int X);
int Judge_Ele(void);
float Cha_BI_He4(float Date_0,float Date_1,float Date_2,float Date_3,float A,float B);


#endif /* CODE_ADC_CONTROL_H_ */
