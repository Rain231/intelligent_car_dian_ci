/***************************************************************************
***�����ֵ50000���ڵ��ռ�ձ�ʱ������Ҫ�����ִ�1000���ٶȲŲ�����********
***************************************************************************/
#include "headfile.h"
#include "steer.h"
#include "motor.h"

/********����ͱ���������********/
//-------�����˲�--------//


uint16 temp_speed_L[3]={0};//���������˲�
uint16 temp_speed_R[3]={0};//�ҵ�������˲�

//-------������PWM--------//
uint32 g_nLeftPWM=0;
uint32 g_nRighPWM=0;

int CODER = 4096;//���������Ϊ512��
float Temp_Orr = 0;
float steer_Angle =0 ;
int16 a = 0;
int16 FeedBack_L,FeedBack_R, speed_R,sheding_L ,sheding_R ;//����������ֵ������ֵ
                                                                //1us����һ��ֵ��360�ȷֳ�4096�ݣ��ٶȼ��㣺����ֵ/4096��*3.14*0.065*1000000��m/s)
int16 left_Speed_last,right_Speed_last, left_Now_Speed,right_Now_Speed;//����������ֵ�ϴ��뱾��
uint32 Motor_L; //���������ٶ�PWM��ʼֵ
uint32 Motor_R;//�����ҵ���ٶ�PWM��ʼֵ
int Hst_speed ;

float error_L=0,d_error_L=0,dd_error_L=0,D_error_L=0,DD_error_L=0; //����ƫ���趨����ǰ���ϴΣ����ϴΡ��м����
float error_R=0,d_error_R=0,dd_error_R=0,D_error_R=0,DD_error_R=0; //�ҵ��ƫ���趨����ǰ���ϴΣ����ϴΡ��м����

int H_speed_L ,H_speed_R ,Hst_speed; //����������٣������

float cycle=10;                      //�ɸ�
float MotorP =0.23,MotorI =0.32,MotorD =0.07; //������PID��ֵ  0.2  0.31  0.31 // 0.23 0.32 0.07
//float MotorP =0.5,MotorI =0,MotorD =0; //������PID��ֵ  0.2  0.31  0.31 // 0.23 0.32 0.07



void SpeedControl(void)
{
  //---�ٶȼ�¼
  left_Speed_last=FeedBack_L;
  right_Speed_last=FeedBack_R;
  
  FeedBack_L = -qtimer_quad_get(QTIMER_1,QTIMER1_TIMER0_C0 ); //������Ҫע��ڶ������������дA������
  FeedBack_R = qtimer_quad_get(QTIMER_1,QTIMER1_TIMER2_C2 ); //��ȡFTM �������� ��������(������ʾ������)
  
  //---���������¼
  left_Now_Speed =(FeedBack_L+left_Speed_last)>>1;//����2
  right_Now_Speed =(FeedBack_R+right_Speed_last)>>1;
  
   qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER0_C0 );
   qtimer_quad_clear(QTIMER_1,QTIMER1_TIMER2_C2 );
  //speed_filter();       //����˲�
  
   
   Hst_speed =gpio_get(SW1)?220:160;  //����
   
  H_speed_L = Hst_speed;
  H_speed_R = Hst_speed;
  
  sheding_L = H_speed_L - (int16)(DirectionError_level*DirectionError_level*cycle); //����� ��ȥƫ����η�����һ��ϵ������Ϊ��ǰ�趨ֵ 40
  if (sheding_L <0) sheding_L = 0;//����� ��ȥƫ����η�����һ��ϵ������Ϊ��ǰ�趨ֵ
  sheding_R = H_speed_R - (int16)(DirectionError_level*DirectionError_level*cycle); //����� ��ȥƫ����η�����һ��ϵ������Ϊ��ǰ�趨ֵ 40
  if (sheding_R<0) sheding_R = 0;
  
#if 1
//  sheding_L = H_speed_L ;
//  if (sheding_L <0) sheding_L = 0;//����� ��ȥƫ����η�����һ��ϵ������Ϊ��ǰ�趨ֵ
//  sheding_R = H_speed_R ;
//  if (sheding_R<0) sheding_R = 0;
  
  steer_Angle = steer_mid - Steer_Out ;
  if(steer_Angle>10) //����
  {
    a=(int16)(100*(steer_Angle)*1.0/(steer_left - steer_right));  //angle_max   45
    if(a>45)  a=45;
    if(a<0)   a=0;//��һ���������ԼӴ���٣��ڶ�������������ǰ����,�������������Ըı������ֵĲ��ٴ�С��ֵ
    Temp_Orr = tan((a*3.14)/180) * 30 / 35;
    sheding_L = (int16)(1.0 * sheding_L * (1.0 + 2.0 * Temp_Orr)); //�Գ����ز���ϵ��0.964
    sheding_R = (int16)(1.0 * sheding_R * (1.0 - 2.0 * Temp_Orr)); 
  }
  else if(steer_Angle<-10)//����
  {
    a=(int16)(100*(-steer_Angle)*1.0/(steer_left - steer_right)); //angle_max   45
    if(a>45)  a=45;
    if(a<0)   a=0;
    Temp_Orr = tan((a*3.14)/180) * 30 / 35;
    sheding_L = (int16)(1.0 * sheding_L * (1.0 - 2.0 * Temp_Orr)); //1.1 ������1.2 ���е�Ư��
    sheding_R = (int16)(1.0 * sheding_R * (1.0 + 2.5 * Temp_Orr));//0.88
  }
#endif
  
  //speedL1();
 // speedR1();
 // PulseCountMeansure();                              //�����ۼ������־λ
  
}

/*****�������ʽPID����1��*******/                                                                  //                       ��λ���
void speedL1(void)
{   
  
  error_L =  sheding_L -FeedBack_L;   //�����ٶ��뵱ǰ�����ٶȵĲ�ֵ
  d_error_L = error_L - D_error_L;        //��ǰ�ٶȲ����ϴ��ٶȲ�Ĳ�ֵ                              
  dd_error_L = d_error_L - DD_error_L;    //�ϴ��ٶȲ������ϴ��ٶȲ�Ĳ�ֵ
  D_error_L = error_L;
  DD_error_L = d_error_L;
  
  
  if((error_L >= 25)||(error_L <= -30)) //�������ƣ����ٶȲ�ֵ����һ��ֵ��������ٻ��߼��٣����ֶ�������С���PWMֵ
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
    Motor_L = gpio_get(SW1)?13000:11000;                                                        //��λ���
    Motor_L = Motor_L + (int16)(MotorP*d_error_L + MotorI*error_L + MotorD*dd_error_L)/10;  //P I D �����ɵ���
      if( gpio_get(SW1)==0)
      { 
        if( Motor_L>11000)
        Motor_L =11000;
        else if(Motor_L<=0)  
        Motor_L = 0; //������СPWMֵ����ֹ��ת
      }
      else if( gpio_get(SW1)==1)
      {    
        if( Motor_L>14000)
        Motor_L =14000;
        else if(Motor_L<=0)  
        Motor_L = 0; //������СPWMֵ����ֹ��ת
        
       }
    
  }
  //return Motor_L; 
}
/*****���2����ʽPID����*******/
void speedR1(void)
{
  error_R =  sheding_R - FeedBack_R;      //�����ٶ��뵱ǰ�����ٶȵĲ�ֵ
  d_error_R = error_R - D_error_R;        //��ǰ�ٶȲ����ϴ��ٶȲ�Ĳ�ֵ                              
  dd_error_R = d_error_R - DD_error_R;    //�ϴ��ٶȲ������ϴ��ٶȲ�Ĳ�ֵ
  D_error_R = error_R;
  DD_error_R = d_error_R;
 if((error_R >= 30)||(error_R <= -30)) //�������ƣ����ٶȲ�ֵ����һ��ֵ��������ٻ��߼��٣����ֶ�������С���PWMֵ
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
   
     // if( blockprognosis == 1 || blockpavement == 1)                                                   ��λ���
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
    Motor_R = gpio_get(SW1)?13000:11000;                                                                    //��λ���
    Motor_R = Motor_R + (int16)(MotorP*d_error_R + MotorI*error_R + MotorD*dd_error_R)/10;  //P I D �����ɵ���                           ��λ���
    if( gpio_get(SW1)==0)
      { 
        if( Motor_R>11000)
        Motor_R =11000;
        else if(Motor_R<=0)  
        Motor_R = 0; //������СPWMֵ����ֹ��ת
      }
      else if( gpio_get(SW1)==1)
      {    
        if( Motor_R>14000)
        Motor_R =14000;
        else if(Motor_R<=0)  
        Motor_R = 0; //������СPWMֵ����ֹ��ת
        
       }
   }
  //return Motor_R; 
}

//////////////////

/*
//�������1��
void chasu1(void)
{
  int end =7; //90
  int B = 1;
  if(abs(DirectionError_level)>0.17)
{ 
    if( DirectionError_level < 0)   //��ת
  {
    DirectionError_level = -DirectionError_level;
    H_speed_L = H_speed_L*(1 + B*(DirectionError_level)/(end));
    H_speed_R = H_speed_R*(1 -B*(DirectionError_level)/(end));
  }   
  else if (DirectionError_level > 0)  //��ת
  {
     H_speed_L = (int) (H_speed_L*(1 - B*(DirectionError_level)/(end)));
     H_speed_R = (int) (H_speed_R*(1+B*(DirectionError_level)/(end)));
  }
}
}
*/



/*******************************************************************************
�������ƣ�speed_filter
�������ܣ������˲�����
����˵����
*******************************************************************************/  
void speed_filter(void)        //�ú�������ʱ 11.7us
{
  
  //�����˲�
  ///////ȡǰ��ɼ������м�ֵ
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
  //�ҵ���˲�
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
