
#include "headfile.h"
#include "adc-control.h"
#include "steer.h"
#include "motor.h"



int main(void)
{
  
    
    uint16 shangweiji_i=0;  
    DisableGlobalIRQ();
     
    board_init();   //��ر��������������ڳ�ʼ��MPU ʱ�� ���Դ���
    
	systick_delay_ms(100);	//��ʱ100ms���ȴ��������������ϵ�ɹ�
    pwm_init(S_MOTOR_PIN,50,3750);//�����ʼ��

 
    ips200_init();//ips����ʼ��
    //ips200_showstr(0,1,"freescale");
    mt9v03x_csi_init();

    
    //��ʼ�����PWM����
    //�����������У�����������Ƶ��ѡ��13K-17K
    //���ռ�ձ�ֵPWM_DUTY_MAX ������fsl_pwm.h�ļ����޸� Ĭ��Ϊ50000
    //����һ��PWMģ�� ����������ͨ��ֻ�����Ƶ��һ�� ռ�ձȲ�һ���� PWM RT1021ֻ������PWMģ�� ÿ��ģ����8��ͨ��
    pwm_init(MOTOR1_A,17000,0);
    pwm_init(MOTOR1_B,17000,0);
    pwm_init(MOTOR2_A,17000,0);
    pwm_init(MOTOR2_B,17000,0);
    //��г�ʼ��
    adc_init(POWER_ADC1_MOD,POWER_ADC1_PIN,ADC_10BIT);  
    adc_init(POWER_ADC2_MOD,POWER_ADC2_PIN,ADC_10BIT);  
    adc_init(POWER_ADC3_MOD,POWER_ADC3_PIN,ADC_10BIT);  
    adc_init(POWER_ADC4_MOD,POWER_ADC4_PIN,ADC_10BIT);
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER0_C0,QTIMER1_TIMER1_C1);
    qtimer_quad_init(QTIMER_1,QTIMER1_TIMER2_C2,QTIMER1_TIMER3_C24);
     seekfree_wireless_init();
    EnableGlobalIRQ(0); 
  
      while(1)
    {
      
      if(mt9v03x_csi_finish_flag)
      {
       //ips200_displayimage032(mt9v03x_csi_image[0],MT9V03X_CSI_W, MT9V03X_CSI_H);
        mt9v03x_csi_finish_flag = 0;
      }
        
      
       Get_AD();
       ips_adcprint();
       
       pwm_duty(S_MOTOR_PIN,steer1());
         ips200_showstr(55,0,"duoji");
         ips200_showstr(100,0,"duoji");

       ips200_showuint16(0,0,(unsigned short int)PWM);
       SpeedControl();
       speedL1();
       speedR1();//����
       speed_filter();//�������˲�
       speed_print();
      pwm_duty(MOTOR1_A, Motor_L);//���1
      pwm_duty(MOTOR1_B, 0);
      pwm_duty(MOTOR2_A, Motor_R);//���2
      pwm_duty(MOTOR2_B, 0);
      if(shangweiji_i==200)
      { 
        shangweiji_i=0;
         shangweiji();
      }
     shangweiji_i++;
   }
}