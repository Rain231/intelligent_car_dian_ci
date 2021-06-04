/*
 * steer.c
 *
 *  Created on: 2021年3月13日
 *      Author: dade
 */
//舵机在逐飞k60上的引脚 PTA9  FTM1_CH1
//测静态p值，在直道上测试，p值越大，舵机越晃，减小p值，直到舵机不晃
//测动态p值，90度弯道测试，d值越大，舵机打脚值越小，找一个合适的值
//p值是油门，d值是刹车
#include "headfile.h"  //逐飞自己的库和自己写的h头文件都在这里面
#include "steer.h"
/****************************舵  机  参  数*******************************/
uint16 steer_left=800; //舵机左边极限值，调整舵机FTM占空比精度时，这个需要调整//1880
uint16 steer_right=480;//舵机右边极限值，调整舵机FTM占空比精度时，这个需要调整 1360
uint16 steer_mid=635;  //舵机居中值5200
int PWM=1000; //舵机输出值，用于控制舵机
int Steer_Out;//舵机输出值，用于弯道两电机差速
/********************************P  I  D  参  数********************************/
float Kp=100,Basic_p_wandao_3=120,Kd=-20*1.5,Basic_p;
float Kp_1,Kp_0;//800舵机设定的初始PD值，只是做初始用，后面使用的是动态P,定值D
float Basic_p_tingbai_1=120,Basic_p_zhidao_2=50;
float Error=0,Last_error=0,Kd_error;//偏差设定,当前、上次、上上次、中间变量
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
/********************************环 岛 判 断 参 数********************************/
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

   if(cr_flag1_r==1 && Flag_Round==0 && R_huan==1 && L_huan==0)                      //右环舵机控制
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3
        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2+50);

  }
   if( cr_flag1_l==1 && Flag_Round==0 && L_huan==1 && R_huan==0)                        //左环舵机控制
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2-50);
  }
   if( cr_flag1_l==1&& Flag_Round==1 && L_huan==1 && R_huan==0)                           //左环舵机控制
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2-10);

  }
   if( cr_flag1_r==1 && Flag_Round==1 && L_huan==0 && R_huan==1)                        //右环舵机控制
  {
       Kp_0=-(40+(DirectionError_level*DirectionError_level))/5;//5  3

        PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*PID_rain.D2+10);
  }
   if( cr_flag1_l==0&&cr_flag1_r==0&& Flag_Round==1 && L_huan==1 && R_huan==0)           //左环舵机控制
     {


            PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);

     }
      if( cr_flag1_l==0&&cr_flag1_r==0 && Flag_Round==1 && L_huan==0 && R_huan==1)//右环舵机控制
     {


                     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);

     }
   if(tflag==1&&sflag==0)//第一次判断三叉
  {

      PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd+80);
 }else if(oflag==1&&sflag==2&&tflag==1)            //第一次，进三叉
         PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);
 else if(tflag==3&&sflag==0&&oflag==2)//第二次进
     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd-80);
 else if(oflag==3&&sflag==2&&tflag==3)            //第二次进，清标志位
     PWM=(int)(steer_mid +DirectionError_level*Kp_0+DirectionError_dot_level*Kd);
 else if(sflag==1&&(tflag==1||tflag==3))
 {//第二次 出
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
    {//防止PWM值越界
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
      {																//防止PWM值越界
        PWM=steer_left;
      }
       if(PWM<steer_right)
       {
        PWM=steer_right;
       }
       return  PWM;


}
