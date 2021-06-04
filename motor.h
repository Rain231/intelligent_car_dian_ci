/*
 * motor.h
 *
 *  Created on: 2021年3月13日
 *      Author: dade
 */
#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#include "include.h"


extern int slow_speed;

extern float temp_5;
extern short int g_nLeftpulse;
extern short int g_nRighpulse;
extern long  int nLeftPWM;
extern long  int nRighPWM;
extern float g_fRealSpeed;
extern float g_fLeftRealSpeed;
extern float g_fRighRealSpeed;
extern float g_fSpeedFilter;
extern float g_fExpectSpeed;
extern float fSpeedErrorInteg;

extern float g_speedControl_P;      //差速比例
extern float g_speedControl_I;      //位移比例
extern float g_fExpectSpeed;
extern int    cr_flag1;//环岛
extern void SpeedControl(void);

extern int MOTOR_DEAD_VAL_L;
extern int MOTOR_DEAD_VAL_R;
extern void dianji(void);
extern float  ad[5];
extern float abs_f(float k);

extern int8 h;
extern int32 Motor_L;
extern void SpeedPID1(void);

extern void speed_filter(void);

 float ABC(float Date_1,float Date_2,float Date_3,int X);
 extern int Slow_Speed(void);



#endif /* CODE_MOTOR_H_ */
