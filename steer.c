//��������k60�ϵ����� PTA9  FTM1_CH1
//�⾲̬pֵ����ֱ���ϲ��ԣ�pֵԽ�󣬶��Խ�Σ���Сpֵ��ֱ���������
//�⶯̬pֵ��90��������ԣ�dֵԽ�󣬶�����ֵԽС����һ�����ʵ�ֵ
//pֵ�����ţ�dֵ��ɲ��
#include "headfile.h"  //����Լ��Ŀ���Լ�д��hͷ�ļ�����������
#include "steer.h"
/****************************��  ��  ��  ��*******************************/
uint16 steer_left=4150; //�����߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ����//1880
uint16 steer_right=3300;//����ұ߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ���� 1360
uint16 steer_mid=3750;  //�������ֵ 
int PWM; //������ֵ�����ڿ��ƶ��
int Steer_Out;//������ֵ������������������
/********************************P  I  D  ��  ��********************************/
float Kp,Basic_p=750,Kd=45;  //800����趨�ĳ�ʼPDֵ��ֻ������ʼ�ã�����ʹ�õ��Ƕ�̬P,��ֵD
float Error=0,Last_error=0,Kd_error;//ƫ���趨,��ǰ���ϴΡ����ϴΡ��м����
float DirectionError_level=0;
float DirectionError_dot_level=0;
float DirectionError_vertical=0;
float DirectionError_dot_vertical=0;

/********************************�� �� �� �� �� ��********************************/


int steer1(void)
{
  DirectionError_level=temp_1/1000;
  DirectionError_dot_level=temp_2/1000;
  DirectionError_vertical=temp_3/1000;
  DirectionError_dot_vertical=temp_4/1000;
  //////�����������ͨ������p,dֵ
     

     Kp=Basic_p+(DirectionError_level*DirectionError_level)/3;//5
    PWM=(int)(steer_mid +DirectionError_level*Kp+DirectionError_dot_level*Kd); //ˮ ƽ �� �� �� ��
 
  
  if(PWM>steer_left) //��ֹPWMֵԽ��
    PWM=steer_left;
  else if(PWM<steer_right)
    PWM=steer_right;
    Steer_Out=PWM;
  //return  steer_left; 
  //return  steer_right; 
  //return  steer_mid; 
  return  PWM;
}