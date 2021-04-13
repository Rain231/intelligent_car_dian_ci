
#include "headfile.h"
#include "adc-control.h"
#include "steer.h"
#include "motor.h"



int main(void)
{
  
    
    uint16 shangweiji_i=0;  
    DisableGlobalIRQ();
     
    board_init();   //务必保留，本函数用于初始化MPU 时钟 调试串口
    
	systick_delay_ms(100);	//延时100ms，等待主板其他外设上电成功
    pwm_init(S_MOTOR_PIN,50,3750);//舵机初始化

 
    ips200_init();//ips屏初始化
    //ips200_showstr(0,1,"freescale");
    mt9v03x_csi_init();

    
    //初始化电机PWM引脚
    //桌大大的推文中，建议电磁组电机频率选用13K-17K
    //最大占空比值PWM_DUTY_MAX 可以在fsl_pwm.h文件中修改 默认为50000
    //对于一个PWM模块 包含的所有通道只能输出频率一样 占空比不一样的 PWM RT1021只有两个PWM模块 每个模块有8个通道
    pwm_init(MOTOR1_A,17000,0);
    pwm_init(MOTOR1_B,17000,0);
    pwm_init(MOTOR2_A,17000,0);
    pwm_init(MOTOR2_B,17000,0);
    //电感初始化
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
       speedR1();//差速
       speed_filter();//编码器滤波
       speed_print();
      pwm_duty(MOTOR1_A, Motor_L);//电机1
      pwm_duty(MOTOR1_B, 0);
      pwm_duty(MOTOR2_A, Motor_R);//电机2
      pwm_duty(MOTOR2_B, 0);
      if(shangweiji_i==200)
      { 
        shangweiji_i=0;
         shangweiji();
      }
     shangweiji_i++;
   }
}