#ifndef _ADC_CONTROL_H
#define _ADC_CONTROL_H

#define POWER_ADC1_MOD  ADC_1       //����ͨ��һ ADCģ���
#define POWER_ADC1_PIN  ADC1_CH3_B14//����ͨ��һ ADC����
                                                     
#define POWER_ADC2_MOD  ADC_1       //����ͨ���� ADCģ���
#define POWER_ADC2_PIN  ADC1_CH4_B15//����ͨ���� ADC����

#define POWER_ADC3_MOD  ADC_1       //����ͨ���� ADCģ���
#define POWER_ADC3_PIN  ADC1_CH10_B21//����ͨ���� ADC����

#define POWER_ADC4_MOD  ADC_1       //����ͨ���� ADCģ���
#define POWER_ADC4_PIN  ADC1_CH12_B23//����ͨ���� ADC����




/*********������ƺ���********************/

extern void Get_AD(void);
extern float  ad[4];
extern float DirectionError[2];	//��DirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ�
extern float DirectionError_dot[2];   //��DirectionError_dot[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��΢�֣�
//��DirectionError_dot[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ��΢�֣�
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
/************ips��ʾ����*********/

extern void ips_adcprint(void);
#endif