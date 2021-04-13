

#ifndef _MOTOR_H
#define _MOTOR_H

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
extern void Get_Speed(void);
extern void SpeedControl(void);
extern void process_moto_pwm(void);

extern int MOTOR_DEAD_VAL_L;
extern int MOTOR_DEAD_VAL_R;
extern void dianji(void);

extern unsigned long int  Motor_L;
extern unsigned long int  Motor_R;
extern void speed(void);
extern void SpeedControl(void);
extern void speedL1(void);
extern void speedR1(void);
extern void speedL2(void);
extern void speedR2(void);
extern void speed_filter(void);
extern void speed_print();

#endif




