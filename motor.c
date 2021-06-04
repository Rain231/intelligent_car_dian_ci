/*
 * motor.c
 *
 *  Created on: 2021��3��13��
 *      Author: dade
 */
/***************************************************************************
***�����ֵ50000���ڵ��ռ�ձ�ʱ������Ҫ�����ִ�1000���ٶȲŲ�����********
***************************************************************************/
//#include "headfile.h"
//#include "steer.h"
#include "motor.h"


/********����ͱ���������********/
//-------�����˲�--------//
uint16 temp_speed_L[3]={0};//���������˲�
uint16 temp_speed_R[3]={0};//�ҵ�������˲�
int8 h;

int32 FeedBack_L,FeedBack_R;                                                             //1us����һ��ֵ��360�ȷֳ�4096�ݣ��ٶȼ��㣺����ֵ/4096��*3.14*0.065*1000000��m/s)

int32 Motor_L ; //���������ٶ�PWM��ʼֵ

uint8 text[1];

float turn1[4]={3.8,0.001,0, 500};
float *p1= turn1;

//�ٶȿ���
void SpeedPID1(void)
{

    FeedBack_L=gpt12_get(GPT12_T2); //������Ҫע��ڶ������������дA������
    gpt12_clear(GPT12_T2);

    Motor_L =  PlacePID_Control(&TurnPID, p1, FeedBack_L, 100);           //λ��ʽpid

if(tflag==1&&sflag==0)//��һ���ж�����
{

    Motor_L = 0;
}else if(oflag==1&&sflag==2&&tflag==1)            //��һ�Σ�������
    Motor_L = PlacePID_Control(&TurnPID, p1, FeedBack_L, 90);
else if(tflag==3&&sflag==0&&oflag==2)//�ڶ��ν�
  Motor_L = 0;
else if(oflag==3&&sflag==2&&tflag==3)            //�ڶ��ν������־λ
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
          Motor_L =3000; //����PWMֵ����ֹԽ��
        else if(Motor_L<=0)
          Motor_L = 0; //������СPWMֵ����ֹ��ת

      h=gpio_get(P33_4);//��λ����

      if(h==0)
        {
            if(0<=Motor_L) //���1   ��ת ����ռ�ձ�Ϊ �ٷ�֮ (1000/GTM_ATOM0_PWM_DUTY_MAX*100)
               {
                   pwm_duty(MOTOR_A, Motor_L);
                   pwm_duty(MOTOR_B, 0);
               }
               else                //���1   ��ת
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
    ips200_showint16(0,IPS_colum+1,icm_gyro_x);                             //��ʾһ��16λ�޷�������
    ips200_showint16(0,IPS_colum+2,icm_gyro_y);                             //��ʾһ��16λ�޷�������
    ips200_showint16(0,IPS_colum+3,icm_gyro_z);                             //��ʾһ��16λ�޷�������
    ips200_showint16(0,IPS_colum+4,icm_acc_x);                             //��ʾһ��16λ�޷�������
    ips200_showint16(0,IPS_colum+5,icm_acc_y);                             //��ʾһ��16λ�޷�������
    ips200_showint16(0,IPS_colum+6,icm_acc_z);                             //��ʾһ��16λ�޷�������
    ips200_showstr(100,IPS_colum+1,"icm_gyro_x");                //��ʾ�ַ���
    ips200_showstr(100,IPS_colum+2,"icm_gyro_y");                //��ʾ�ַ���
    ips200_showstr(100,IPS_colum+3,"icm_gyro_z");                //��ʾ�ַ���
    ips200_showstr(100,IPS_colum+4,"icm_acc_x");                //��ʾ�ַ���
    ips200_showstr(100,IPS_colum+5,"icm_acc_y");                //��ʾ�ַ���
    ips200_showstr(100,IPS_colum+6,"icm_acc_z");                //��ʾ�ַ���
    */
}


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

void dianji()//��λ����ʾ
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
