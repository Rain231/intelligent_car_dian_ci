/*
 * steer.h
 *
 *  Created on: 2021年3月13日
 *      Author: dade
 */
#ifndef CODE_STEER_H_
#define CODE_STEER_H_
#include "include.h"

extern int steer1(void);
extern int PWM; //舵机输出值
extern int Steer_Out;//舵机输出值
//extern  float Error,steer_P,Basic_p;
//extern float xiuzheng,cxiu;
extern float line;
extern float DirectionError_level;
extern unsigned short int steer_left; //舵机左边极限值，调整舵机FTM占空比精度时，这个需要调整//1880
extern unsigned short int steer_right;//舵机右边极限值，调整舵机FTM占空比精度时，这个需要调整 1360
extern unsigned short int steer_mid;  //舵机居中值
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
