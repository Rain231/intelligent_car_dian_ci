#include "headfile.h"  //����Լ��Ŀ���Լ�д��hͷ�ļ�����������
extern int steer1();
extern int PWM; //������ֵ
extern int Steer_Out;//������ֵ
//extern  float Error,steer_P,Basic_p;
//extern float xiuzheng,cxiu;
extern float line;
extern float DirectionError_level;
extern uint16 steer_left; //�����߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ����//1880
extern uint16 steer_right;//����ұ߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ���� 1360
extern uint16 steer_mid;  //�������ֵ 
extern float temp_1;  //temp_1 = DirectionError[0]*1000;
extern float temp_2;  //temp_2 = DirectionError_dot[0]*1000;
extern float temp_3;  //temp_3 = DirectionError[1]*1000;
extern float temp_4;  //temp_4 = DirectionError_dot[1]*1000;
extern void steer_init();
extern float ad1;
 extern float ad0;
 extern int    In_Flag_Right;
extern int    cr_flag1;
extern uint8    Flag_Round;
extern int    L_huan;
extern int    R_huan;