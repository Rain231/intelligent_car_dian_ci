/*
 * steer.c
 *
 *  Created on: 2021��3��13��
 *      Author: dade
 */
//��������k60�ϵ����� PTA9  FTM1_CH1
//�⾲̬pֵ����ֱ���ϲ��ԣ�pֵԽ�󣬶��Խ�Σ���Сpֵ��ֱ���������
//�⶯̬pֵ��90��������ԣ�dֵԽ�󣬶�����ֵԽС����һ�����ʵ�ֵ
//pֵ�����ţ�dֵ��ɲ��
#include "headfile.h"  //����Լ��Ŀ���Լ�д��hͷ�ļ�����������
#include "steer.h"
/****************************��  ��  ��  ��*******************************/
uint16 steer_left=800; //�����߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ����//1880
uint16 steer_right=480;//����ұ߼���ֵ���������FTMռ�ձȾ���ʱ�������Ҫ���� 1360
uint16 steer_mid=635;  //�������ֵ5200
int PWM=1000; //������ֵ�����ڿ��ƶ��
int Steer_Out;//������ֵ������������������
/********************************P  I  D  ��  ��********************************/
float Kp=100,Basic_p_wandao_3=120,Kd=-20*1.5,Basic_p;
float Kp_1,Kp_0;//800����趨�ĳ�ʼPDֵ��ֻ������ʼ�ã�����ʹ�õ��Ƕ�̬P,��ֵD
float Basic_p_tingbai_1=120,Basic_p_zhidao_2=50;
float Error=0,Last_error=0,Kd_error;//ƫ���趨,��ǰ���ϴΡ����ϴΡ��м����
float DirectionError_level=0;
float DirectionError_dot_level=0;
float DirectionError_vertical=0;
float DirectionError_dot_vertical=0;
int D=1;
int S;
float pwm_zhi[5]={0};
float pwm_error;
int  PWMS[14];
int p=0;
int ad4[14]={0};
/********************************�� �� �� �� �� ��********************************/
//int cr_flag1_l,cr_flag1_r;
int    cr_flag1;

int steer1()
{
  DirectionError_level=temp_1/100;
  DirectionError_dot_level=temp_2/1000;
  DirectionError_vertical=temp_3/100;
  DirectionError_dot_vertical=temp_4/1000;


  //Basic_p=Basic_p_wandao_3+D;

  PID_rain.D2=-20;
if(DirectionError[0]*100>10)
{
    Basic_p=40*3;Kd=-20*2;
}else
{
    Basic_p=40*3;Kd=-20*1.5;
}


  Kp_0=-(Basic_p+(DirectionError_level*DirectionError_level))/5;//5  3

  if((cr_flag1_l==0||cr_flag1_r==0)&&sflag==0)
  {

      PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);

  }

   if(cr_flag1_r==1 && Flag_Round==0 && R_huan==1 && L_huan==0)                      //�һ��������
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3
        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2+50);

  }
   if( cr_flag1_l==1 && Flag_Round==0 && L_huan==1 && R_huan==0)                        //�󻷶������
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2-50);
  }
   if( cr_flag1_l==1&& Flag_Round==1 && L_huan==1 && R_huan==0)                           //�󻷶������
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2-10);

  }
   if( cr_flag1_r==1 && Flag_Round==1 && L_huan==0 && R_huan==1)                        //�һ��������
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2+10);
  }
   if( cr_flag1_l==0&&cr_flag1_r==0&& Flag_Round==1 && L_huan==1 && R_huan==0)           //�󻷶������
     {


            PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);

     }
      if( cr_flag1_l==0&&cr_flag1_r==0 && Flag_Round==1 && L_huan==0 && R_huan==1)//�һ��������
     {


                     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);

     }
   if(tflag==1&&sflag==0)//��һ���ж�����
  {

      PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd+80);
 }else if(oflag==1&&sflag==2&&tflag==1)            //��һ�Σ�������
         PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);
 else if(tflag==3&&sflag==0&&oflag==2)//�ڶ��ν�
     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd-80);
 else if(oflag==3&&sflag==2&&tflag==3)            //�ڶ��ν������־λ
     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);
 else if(sflag==1&&(tflag==1||tflag==3))
 {//�ڶ��� ��
   PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);
 }
   /*
  PWMS[5]=PWMS[4];
  PWMS[4]=PWMS[3];
  PWMS[3]=PWMS[2];
  PWMS[2]=PWMS[1];
  PWMS[1]=PWMS[0];
  PWMS[0]=PWM;*/

        ad4[1]=ad4[0];
        ad4[0]=ad[4];
    Judge_Lose_Line();

    if(PWM>steer_left)
    {//��ֹPWMֵԽ��
      PWM=steer_left;
    }
     if(PWM<steer_right)
     {
      PWM=steer_right;
     }
     Steer_Out=PWM;
    //return  steer_left;
    //return  steer_right;
    //return  steer_mid;
    return  PWM;



  }
int lost_line;
uint16 Judge_Lose_Line(void)
{
	if(ad[0]-ad[3]<50&&ad[0]-ad[3]>-50)
{
    lost_line=1;
}
else
{
   lost_line=0;
}
if(lost_line==1)
    {
    if((ad[0]-ad[3])>0)
    {
        PWMS[0]=PWM;
        PWM+=20;

    }
    else
        PWM-=20;
    /*PWM=PWMS[0]+PWMS[1]+PWMS[2]+PWMS[3]+PWMS[4]+PWMS[5];
        PWM=PWM/6;
        PWMS[0]=PWM;*/
     }




    if(PWM>steer_left)
      {																//��ֹPWMֵԽ��
        PWM=steer_left;
      }
       if(PWM<steer_right)
       {
        PWM=steer_right;
       }
       return  PWM;


}
