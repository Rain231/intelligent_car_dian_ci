/***************************************************************************
***以最大值50000调节电机占空比时，右轮要比左轮大1000，速度才差不多相等********
***************************************************************************/
#include "headfile.h"
#include "steer.h"
#include "motor.h"

/********电机和编码器参数********/
//-------脉冲滤波--------//


uint16 temp_speed_L[3]={0};//左电机脉冲滤波
uint16 temp_speed_R[3]={0};//右电机脉冲滤波

//-------左右轮PWM--------//
uint32 g_nLeftPWM=0;
uint32 g_nRighPWM=0;

int CODER = 4096;//定义编码器为512线
float Temp_Orr = 0;
float steer_Angle =0 ;
int16 a = 0;
int16 FeedBack_L,FeedBack_R, speed_R,sheding_L ,sheding_R ;//编码器反馈值、期望值
                                                                //1us返回一次值，360度分成4096份，速度计算：（数值/4096）*3.14*0.065*1000000（m/s)
int16 left_Speed_last,right_Speed_last, left_Now_Speed,right_Now_Speed;//编码器反馈值上次与本次
uint32 Motor_L; //定义左电机速度PWM初始值
uint32 Motor_R;//定义右电机速度PWM初始值
int Hst_speed ;

float error_L=0,d_error_L=0,dd_error_L=0,D_error_L=0,DD_error_L=0; //左电机偏差设定，当前、上次，上上次、中间变量
float error_R=0,d_error_R=0,dd_error_R=0,D_error_R=0,DD_error_R=0; //右电机偏差设定，当前、上次，上上次、中间变量

int H_speed_L ,H_speed_R ,Hst_speed; //定义电机最高速，最低速

float cycle=10;                      //可改
float MotorP =0.23,MotorI =0.32,MotorD =0.07; //定义电机PID赋值  0.2  0.31  0.31 // 0.23 0.32 0.07
//float MotorP =0.5,MotorI =0,MotorD =0; //定义电机PID赋值  0.2  0.31  0.31 // 0.23 0.32 0.07



void SpeedControl(void)
{
  //---速度记录
  left_Speed_last=FeedBack_L;
  right_Speed_last=FeedBack_R;
  
  FeedBack_L = -qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_C0 ); //这里需要注意第二个参数务必填写A相引脚
  FeedBack_R = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_C2 ); //获取FTM 正交解码 的脉冲数(负数表示反方向)
  
  //---现在脉冲记录
  left_Now_Speed =(FeedBack_L+left_Speed_last)>>1;//除以2
  right_Now_Speed =(FeedBack_R+right_Speed_last)>>1;
  
   qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER0_C0 );
   qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2 );
  //speed_filter();       //电机滤波
  
   
   Hst_speed =gpio_get(SW1)?220:160;  //开关
   
  H_speed_L = Hst_speed;
  H_speed_R = Hst_speed;
  
  sheding_L = H_speed_L - (int16)(DirectionError_level*DirectionError_level*cycle); //最高速 减去偏差二次方除以一个系数，做为当前设定值 40
  if (sheding_L <0) sheding_L = 0;//最高速 减去偏差二次方除以一个系数，做为当前设定值
  sheding_R = H_speed_R - (int16)(DirectionError_level*DirectionError_level*cycle); //最高速 减去偏差二次方除以一个系数，做为当前设定值 40
  if (sheding_R<0) sheding_R = 0;
  
#if 1
//  sheding_L = H_speed_L ;
//  if (sheding_L <0) sheding_L = 0;//最高速 减去偏差二次方除以一个系数，做为当前设定值
//  sheding_R = H_speed_R ;
//  if (sheding_R<0) sheding_R = 0;
  
  steer_Angle = steer_mid - Steer_Out ;
  if(steer_Angle>10) //打右
  {
    a=(int16)(100*(steer_Angle)*1.0/(steer_left - steer_right));  //angle_max   45
    if(a>45)  a=45;
    if(a<0)   a=0;//第一个常数可以加大差速，第二个常数可以提前差速,第三个常数可以改变左右轮的差速大小差值
    Temp_Orr = tan((a*3.14)/180) * 30 / 35;
    sheding_L = (int16)(1.0 * sheding_L * (1.0 + 2.0 * Temp_Orr)); //试车场地差速系数0.964
    sheding_R = (int16)(1.0 * sheding_R * (1.0 - 2.0 * Temp_Orr)); 
  }
  else if(steer_Angle<-10)//打左
  {
    a=(int16)(100*(-steer_Angle)*1.0/(steer_left - steer_right)); //angle_max   45
    if(a>45)  a=45;
    if(a<0)   a=0;
    Temp_Orr = tan((a*3.14)/180) * 30 / 35;
    sheding_L = (int16)(1.0 * sheding_L * (1.0 - 2.0 * Temp_Orr)); //1.1 不超过1.2 ，有点漂移
    sheding_R = (int16)(1.0 * sheding_R * (1.0 + 2.5 * Temp_Orr));//0.88
  }
#endif
  
  //speedL1();
 // speedR1();
 // PulseCountMeansure();                              //脉冲累计清除标志位
  
}

/*****电机增量式PID调节1档*******/                                                                  //                       挡位需改
void speedL1(void)
{   
  
  error_L =  sheding_L -FeedBack_L;   //期望速度与当前反馈速度的差值
  d_error_L = error_L - D_error_L;        //当前速度差与上次速度差的差值                              
  dd_error_L = d_error_L - DD_error_L;    //上次速度差与上上次速度差的差值
  D_error_L = error_L;
  DD_error_L = d_error_L;
  
  
  if((error_L >= 25)||(error_L <= -30)) //棒棒控制，若速度差值大于一定值，急需加速或者减速，则手动给定最小最大PWM值
  {
    if(error_L >= 25)
     {
       if( gpio_get(SW1)==0)
      {  
        Motor_L =11000;
      }
      else if( gpio_get(SW1)==1)
      {
         Motor_L =14000;
       }
      
    } 
    else if(error_L <= -30)
   
      //if( blockprognosis == 1 || blockpavement == 1 )
      //{
      //  Motor_L=5; //160
     // }
     // else
     {
       if( gpio_get(SW1)==0)
      {  
        Motor_L =10000;
      }
      else if( gpio_get(SW1)==1)
      {
         Motor_L =8000;
       }
      
    }  
    
  }
  else
  {
    Motor_L = gpio_get(SW1)?13000:11000;                                                        //挡位需改
    Motor_L = Motor_L + (int16)(MotorP*d_error_L + MotorI*error_L + MotorD*dd_error_L)/10;  //P I D 参数可调，
      if( gpio_get(SW1)==0)
      { 
        if( Motor_L>11000)
        Motor_L =11000;
        else if(Motor_L<=0)  
        Motor_L = 0; //限制最小PWM值，防止疯转
      }
      else if( gpio_get(SW1)==1)
      {    
        if( Motor_L>14000)
        Motor_L =14000;
        else if(Motor_L<=0)  
        Motor_L = 0; //限制最小PWM值，防止疯转
        
       }
    
  }
  //return Motor_L; 
}
/*****电机2增量式PID调节*******/
void speedR1(void)
{
  error_R =  sheding_R - FeedBack_R;      //期望速度与当前反馈速度的差值
  d_error_R = error_R - D_error_R;        //当前速度差与上次速度差的差值                              
  dd_error_R = d_error_R - DD_error_R;    //上次速度差与上上次速度差的差值
  D_error_R = error_R;
  DD_error_R = d_error_R;
 if((error_R >= 30)||(error_R <= -30)) //棒棒控制，若速度差值大于一定值，急需加速或者减速，则手动给定最小最大PWM值
  {
    
     if(error_R >= 30)
     {
       if( gpio_get(SW1)==0)
      {  
        Motor_R =11000;
      }
     
      else if( gpio_get(SW1)==1)
      {
         Motor_R =13000;
       }
     }
    else if(error_R <= -30)
   
     // if( blockprognosis == 1 || blockpavement == 1)                                                   挡位需改
     // { 
      //  error_R=5; //160
      //}
      //else
      {
       if( gpio_get(SW1)==0)
      {  
        Motor_R =10000;
      }
      else if( gpio_get(SW1)==1)
      {
         Motor_R =8000;
       }
      
    }  

  }
 else 
  {
    Motor_R = gpio_get(SW1)?13000:11000;                                                                    //挡位需改
    Motor_R = Motor_R + (int16)(MotorP*d_error_R + MotorI*error_R + MotorD*dd_error_R)/10;  //P I D 参数可调，                           挡位需改
    if( gpio_get(SW1)==0)
      { 
        if( Motor_R>11000)
        Motor_R =11000;
        else if(Motor_R<=0)  
        Motor_R = 0; //限制最小PWM值，防止疯转
      }
      else if( gpio_get(SW1)==1)
      {    
        if( Motor_R>14000)
        Motor_R =14000;
        else if(Motor_R<=0)  
        Motor_R = 0; //限制最小PWM值，防止疯转
        
       }
   }
  //return Motor_R; 
}

//////////////////

/*
//电机差速1档
void chasu1(void)
{
  int end =7; //90
  int B = 1;
  if(abs(DirectionError_level)>0.17)
{ 
    if( DirectionError_level < 0)   //右转
  {
    DirectionError_level = -DirectionError_level;
    H_speed_L = H_speed_L*(1 + B*(DirectionError_level)/(end));
    H_speed_R = H_speed_R*(1 -B*(DirectionError_level)/(end));
  }   
  else if (DirectionError_level > 0)  //左转
  {
     H_speed_L = (int) (H_speed_L*(1 - B*(DirectionError_level)/(end)));
     H_speed_R = (int) (H_speed_R*(1+B*(DirectionError_level)/(end)));
  }
}
}
*/



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
void speed_print()
{
               ips200_showstr(150,3,"FB_L_dj");
         ips200_showstr(250,3,"FB_R_dj");

     ips200_showint16(0,3,FeedBack_L);
     ips200_showint16(80,3,FeedBack_R);
}
