/*
 * steer.h
 *
 *  Created on: 2021��3��13��
 *      Author: dade
 */
#ifndef CODE_STEER_H_
#define CODE_STEER_H_
#include "include.h"

extern int steer1(void);
extern int PWM; //������ֵ
extern int Steer_Out;//������ֵ
//extern  float Error,steer_P,Basic_p;
//extern float xiuzheng,cxiu;
extern float line;
extern float DirectionError_level;
extern unsigned short int steer_left; //�����߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ����//1880
extern unsigned short int steer_right;//����ұ߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ���� 1360
extern unsigned short int steer_mid;  //�������ֵ
extern float temp_1;  //temp_1 = DirectionError[0]*1000;
extern float temp_2;  //temp_2 = DirectionError_dot[0]*1000;
extern float temp_3;  //temp_3 = DirectionError[1]*1000;
extern float temp_4;  //temp_4 = DirectionError_dot[1]*1000;
extern void steer_init();
extern float ad1;
 extern float ad0;
 extern int    In_Flag_Right;
extern int    cr_flag1;

extern int    L_huan;
extern int    R_huan;


uint16 Judge_Lose_Line(void);
#endif /* CODE_STEER_H_ */
