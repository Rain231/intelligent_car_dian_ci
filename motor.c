/*
 * motor.c
 *
 *  Created on: 2021年3月13日
 *      Author: dade
 */
/***************************************************************************
***以最大值50000调节电机占空比时，右轮要比左轮大1000，速度才差不多相等********
***************************************************************************/
//#include "headfile.h"
//#include "steer.h"
#include "motor.h"


/********电机和编码器参数********/
//-------脉冲滤波--------//
uint16 temp_speed_L[3]={0};//左电机脉冲滤波
uint16 temp_speed_R[3]={0};//右电机脉冲滤波
int8 h;

int32 FeedBack_L,FeedBack_R;                                                             //1us返回一次值，360度分成4096份，速度计算：（数值/4096）*3.14*0.065*1000000（m/s)

int32 Motor_L ; //定义左电机速度PWM初始值

uint8 text[1];

float turn1[4]={3.8,0.001,0, 500};
float *p1= turn1;

//速度控制
void SpeedPID1(void)
{

    FeedBack_L=gpt12_get(GPT12_T2); //这里需要注意第二个参数务必填写A相引脚
    gpt12_clear(GPT12_T2);

    Motor_L =  PlacePID_Control(&TurnPID, p1, FeedBack_L, 100);           //位置式pid

if(tflag==1&&sflag==0)//第一次判断三叉
{

    Motor_L = 0;
}else if(oflag==1&&sflag==2&&tflag==1)            //第一次，进三叉
    Motor_L = PlacePID_Control(&TurnPID, p1, FeedBack_L, 90);
else if(tflag==3&&sflag==0&&oflag==2)//第二次进
  Motor_L = 0;
else if(oflag==3&&sflag==2&&tflag==3)            //第二次进，清标志位
    Motor_L =PlacePID_Control(&TurnPID, p1, FeedBack_L, 90);


if(cr_flag1==1&& Flag_Round==0&&(R_huan==1 || L_huan==1))
{
    Motor_L =  PlacePID_Control(&TurnPID, p1, FeedBack_L, 70);
}
else if(Flag_Round==1)
{
    Motor_L =  PlacePID_Control(&TurnPID, p1, FeedBack_L, 90);
}
//Slow_Speed();



    Motor_L =  Slow_Speed();
        if(Motor_L>3000)
          Motor_L =3000; //限制PWM值，防止越界
        else if(Motor_L<=0)
          Motor_L = 0; //限制最小PWM值，防止疯转

      h=gpio_get(P33_4);//挡位控制

      if(h==0)
        {
            if(0<=Motor_L) //电机1   正转 设置占空比为 百分之 (1000/GTM_ATOM0_PWM_DUTY_MAX*100)
               {
                   pwm_duty(MOTOR_A, Motor_L);
                   pwm_duty(MOTOR_B, 0);
               }
               else                //电机1   反转
               {
                   pwm_duty(MOTOR_A, 0);
                   pwm_duty(MOTOR_B, -Motor_L);
               }


        }
        else
        {

            pwm_duty(MOTOR_A, 0);
            pwm_duty(MOTOR_B, 0);
        }

      ips200_showint16(25,4,FeedBack_L);
      ips200_showstr(0,4,"v1:");

}

void GyroInit(void)
{
    get_icm20602_accdata_spi();
    get_icm20602_gyro_spi();
    /*
    ips200_showint16(0,IPS_colum+1,icm_gyro_x);                             //显示一个16位无符号整数
    ips200_showint16(0,IPS_colum+2,icm_gyro_y);                             //显示一个16位无符号整数
    ips200_showint16(0,IPS_colum+3,icm_gyro_z);                             //显示一个16位无符号整数
    ips200_showint16(0,IPS_colum+4,icm_acc_x);                             //显示一个16位无符号整数
    ips200_showint16(0,IPS_colum+5,icm_acc_y);                             //显示一个16位无符号整数
    ips200_showint16(0,IPS_colum+6,icm_acc_z);                             //显示一个16位无符号整数
    ips200_showstr(100,IPS_colum+1,"icm_gyro_x");                //显示字符串
    ips200_showstr(100,IPS_colum+2,"icm_gyro_y");                //显示字符串
    ips200_showstr(100,IPS_colum+3,"icm_gyro_z");                //显示字符串
    ips200_showstr(100,IPS_colum+4,"icm_acc_x");                //显示字符串
    ips200_showstr(100,IPS_colum+5,"icm_acc_y");                //显示字符串
    ips200_showstr(100,IPS_colum+6,"icm_acc_z");                //显示字符串
    */
}


/*******************************************************************************
函数名称：speed_filter
函数功能：脉冲滤波函数
参数说明：
*******************************************************************************/
void speed_filter(void)        //该函数最大耗时 11.7us
{

  //左电机滤波
  ///////取前后采集到的中间值
  temp_speed_L[2]=temp_speed_L[1];
  temp_speed_L[1]=temp_speed_L[0];
  temp_speed_L[0]=FeedBack_L;
  if((temp_speed_L[2]>=temp_speed_L[1]&&temp_speed_L[2]<=temp_speed_L[0])||(temp_speed_L[2]>=temp_speed_L[0]&&temp_speed_L[2]<=temp_speed_L[1]))
    FeedBack_L=temp_speed_L[2];
  else
    if((temp_speed_L[1]>=temp_speed_L[2]&&temp_speed_L[1]<=temp_speed_L[0])||(temp_speed_L[1]>=temp_speed_L[0]&&temp_speed_L[1]<=temp_speed_L[2]))
      FeedBack_L=temp_speed_L[1];
    else
      if((temp_speed_L[0]>=temp_speed_L[2]&&temp_speed_L[0]<=temp_speed_L[1])||(temp_speed_L[0]>=temp_speed_L[1]&&temp_speed_L[0]<=temp_speed_L[2]))
        FeedBack_L=temp_speed_L[0];
  //右电机滤波
  temp_speed_R[2]=temp_speed_R[1];
  temp_speed_R[1]=temp_speed_R[0];
  temp_speed_R[0]=FeedBack_R;
  if((temp_speed_R[2]>=temp_speed_R[1]&&temp_speed_R[2]<=temp_speed_R[0])||(temp_speed_R[2]>=temp_speed_R[0]&&temp_speed_R[2]<=temp_speed_R[1]))
    FeedBack_R=temp_speed_R[2];
  else
    if((temp_speed_R[1]>=temp_speed_R[2]&&temp_speed_R[1]<=temp_speed_R[0])||(temp_speed_R[1]>=temp_speed_R[0]&&temp_speed_R[1]<=temp_speed_R[2]))
      FeedBack_R=temp_speed_R[1];
    else
      if((temp_speed_R[0]>=temp_speed_R[2]&&temp_speed_R[0]<=temp_speed_R[1])||(temp_speed_R[0]>=temp_speed_R[1]&&temp_speed_R[0]<=temp_speed_R[2]))
        FeedBack_R=temp_speed_R[0];

}

void dianji()//上位机显示
{
  uint8 zuo[7]="zuo:";
  uint8 you[7]="you:";
  uint8 txee5[3]="   ";
  zuo[4]=((signed short int) FeedBack_L/100)+48;
  zuo[5]=(((signed short int)FeedBack_L/10)%10)+48;
  zuo[6]=((signed short int)FeedBack_L%10)+48;
  seekfree_wireless_send_buff(zuo,7);
  seekfree_wireless_send_buff( txee5,3);


  you[4]=((signed short int) FeedBack_R/100)+48;
  you[5]=(((signed short int)FeedBack_R/10)%10)+48;
  you[6]=((signed short int)FeedBack_R%10)+48;
  seekfree_wireless_send_buff(you,7);
seekfree_wireless_send_buff( txee5,2);

}

float ABC(float Date_1,float Date_2,float Date_3,int X)
{

    float He = 0;
    float Result;

    He =Date_1+Date_2;
    Result = He/Date_3*X;
    return Result;

}


int Slow_Speed(void)
       {

    float result;
    result = DirectionError[0]*100;
           if(result<10.0)
           {
                return PlacePID_Control(&TurnPID, p1, FeedBack_L, 100);;
           }

       }
