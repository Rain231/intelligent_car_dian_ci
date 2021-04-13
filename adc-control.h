#ifndef _ADC_CONTROL_H
#define _ADC_CONTROL_H

#define POWER_ADC1_MOD  ADC_1       //定义通道一 ADC模块号
#define POWER_ADC1_PIN  ADC1_CH3_B14//定义通道一 ADC引脚
                                                     
#define POWER_ADC2_MOD  ADC_1       //定义通道二 ADC模块号
#define POWER_ADC2_PIN  ADC1_CH4_B15//定义通道二 ADC引脚

#define POWER_ADC3_MOD  ADC_1       //定义通道三 ADC模块号
#define POWER_ADC3_PIN  ADC1_CH10_B21//定义通道三 ADC引脚

#define POWER_ADC4_MOD  ADC_1       //定义通道四 ADC模块号
#define POWER_ADC4_PIN  ADC1_CH12_B23//定义通道四 ADC引脚




/*********方向控制函数********************/

extern void Get_AD(void);
extern float  ad[4];
extern float DirectionError[2];	//（DirectionError[0]为一对水平电感的差比和偏差）
extern float DirectionError_dot[2];   //（DirectionError_dot[0]为一对水平电感的差比和偏差微分）
//（DirectionError_dot[1]为一对垂直电感的差比和偏差微分）
extern void  shangweiji();
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
extern uint8  Flag_Round;
/************ips显示函数*********/

extern void ips_adcprint(void);
#endif