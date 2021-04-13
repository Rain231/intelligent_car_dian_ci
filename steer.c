//舵机在逐飞k60上的引脚 PTA9  FTM1_CH1
//测静态p值，在直道上测试，p值越大，舵机越晃，减小p值，直到舵机不晃
//测动态p值，90度弯道测试，d值越大，舵机打脚值越小，找一个合适的值
//p值是油门，d值是刹车
#include "headfile.h"  //逐飞自己的库和自己写的h头文件都在这里面
#include "steer.h"
/****************************舵  机  参  数*******************************/
uint16 steer_left=4150; //舵机左边极限值，调整舵机FTM占空比精度时，这个需要调整//1880
uint16 steer_right=3300;//舵机右边极限值，调整舵机FTM占空比精度时，这个需要调整 1360
uint16 steer_mid=3750;  //舵机居中值 
int PWM; //舵机输出值，用于控制舵机
int Steer_Out;//舵机输出值，用于弯道两电机差速
/********************************P  I  D  参  数********************************/
float Kp,Basic_p=750,Kd=45;  //800舵机设定的初始PD值，只是做初始用，后面使用的是动态P,定值D
float Error=0,Last_error=0,Kd_error;//偏差设定,当前、上次、上上次、中间变量
float DirectionError_level=0;
float DirectionError_dot_level=0;
float DirectionError_vertical=0;
float DirectionError_dot_vertical=0;

/********************************环 岛 判 断 参 数********************************/


int steer1(void)
{
  DirectionError_level=temp_1/1000;
  DirectionError_dot_level=temp_2/1000;
  DirectionError_vertical=temp_3/1000;
  DirectionError_dot_vertical=temp_4/1000;
  //////环岛里面和普通赛道的p,d值
     

     Kp=Basic_p+(DirectionError_level*DirectionError_level)/3;//5
    PWM=(int)(steer_mid +DirectionError_level*Kp+DirectionError_dot_level*Kd); //水 平 电 感 控 制
 
  
  if(PWM>steer_left) //防止PWM值越界
    PWM=steer_left;
  else if(PWM<steer_right)
    PWM=steer_right;
    Steer_Out=PWM;
  //return  steer_left; 
  //return  steer_right; 
  //return  steer_mid; 
  return  PWM;
}