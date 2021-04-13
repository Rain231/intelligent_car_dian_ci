#include "headfile.h"
#include "adc-control.h"
/********************************************电感分布*****************************************************
ad[0]                    ad[2]                ad[4]                        ad[3]                    ad[1]       

*********************************************************************************************************/
uint8 Flag_Stop;////////////停车标志位
float dirControl_P = 400;//300;//600;	//方向控制P  550 
float dirControl_D = 500;//300;//270;	//方向控制D
float dirControl_P_cr = 1000;	//圆环控制P
float dirControl_D_cr = 3000;	//圆环控制D
 int adcrecognition=0;//电感识别
float DirectionError[2];	//（DirectionError[0]为一对水平电感的差比和偏差）
//（DirectionError[1]为一对垂直电感的差比和偏差）

float DirectionError_dot[2];   //（DirectionError_dot[0]为一对水平电感的差比和偏差微分）
//（DirectionError_dot[1]为一对垂直电感的差比和偏差微分）

float DirectionControlOut;	//方向控制输出
float temp_1=0.0;
float temp_2=0.0;
float temp_3=0.0;
float temp_4=0.0;

float DirectionErrorTemp_1[4]={0,0,0,0};
float DirectionErrorTemp_2[4]={0,0,0,0};
float  ad[4] = {0,0,0,0};
float  ad_temp[4] = {0,0,0,0};
float  ad_temp_1[4] = {0,0,0,0};
int    ad_min[4] = {0,0,0,0};
int    ad_max[4] ={470,480,450,480}; //{435,450,420,435,500}; 
int16  ValueOfADFilter[4]={0};
int16  ValueOfAD[4]={0};	//获取的电感值

uint8  Flag_Round = 0;	
float  Dir_differential_1,Dir_differential_2,temp1;
int    right_motor_val,left_motor_val;
int    cr_Flag_Right=0;
int    cr_Flag_Left=0;
int    In_Flag_Right=0;
int    In_Flag_Left=0;
int    L_huan=0;
int    Y_huan=0;
int    cr_flag1 = 0;
int    cr_flag2 = 0;
int    cr_flag3 = 0;
int    cr_flag_out1 = 0;
int    cr_flag_out2 = 0;
int    cr_flag_out3 = 0;
float    Symbol;
void Get_AD(void)
{
  uint16  get_ad[4][4],temple;
  int16  i,j,k;
  int16  ad_sum[4];
  int16  ValueOfADOld[4],ValueOfADNew[4];
  
  for(i=0;i<4;i++)//需移植
  {
    get_ad[0][i] = adc_mean_filter(POWER_ADC1_MOD,POWER_ADC1_PIN,10);      //ad0     B14
    get_ad[1][i] = adc_mean_filter(POWER_ADC2_MOD,POWER_ADC2_PIN,10);      //ad1     A15
    get_ad[2][i] = adc_mean_filter(POWER_ADC3_MOD,POWER_ADC3_PIN,10);      //ad2     B21
    get_ad[3][i] = adc_mean_filter(POWER_ADC4_MOD,POWER_ADC4_PIN,10);      //ad3     B23      
  }
  ////////////////////////冒泡排序////////////////////////////////
  for(i=0;i<4;i++)   
  {
    for(j=0;j<3;j++)
    {
      for(k=0;k<3-j;k++)
      {
        if(get_ad[i][k] > get_ad[i][k+1])
        {
          temple = get_ad[i][k+1];
          get_ad[i][k+1] = get_ad[i][k];
          get_ad[i][k] = temple;
        }
      }
    }
  }
  for(i=0;i<4;i++)    //求中间三项的和
  {
    ad_sum[i] = get_ad[i][1] + get_ad[i][2] ;       
    ad_temp[i] = ad_sum[i] / 2;
  }
  //滤波  采集梯度平滑，每次采集最大变化40
  for(i=0;i<4;i++)           
  {
       // ValueOfAD[i] = ad_temp[i]; //将数值中个位数除掉

    ValueOfAD[i] = ((int16)(ad_temp[i]/10)*10); //将数值中个位数除掉
    ValueOfADOld[i] = ValueOfADFilter[i];
    ValueOfADNew[i] = ValueOfAD[i];
    
    if(ValueOfADNew[i]>=ValueOfADOld[i])
      ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>20?(ValueOfADOld[i]+20):ValueOfADNew[i]);
    else
      ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-20?(ValueOfADOld[i]-20):ValueOfADNew[i]);
    ad_temp[i]=ValueOfADFilter[i];
  }
  
  for(i = 0; i < 4; i++)
  {
       ad_temp_1[i] = (((int16)ad_temp[i])*10)/10;
           // ad_temp_1[i] = ad_temp[i];   

  }
  
  for(i = 0; i < 4; i++)
  {
    if(ad_temp[i] < ad_min[i])
    {
      ad_temp[i] = ad_min[i];
    }
    if(ad_temp[i] > ad_max[i])
    {
      ad_temp[i] = ad_max[i];
    }
    ad[i] = ((ad_temp[i] - ad_min[i]) / (ad_max[i] - ad_min[i])) * 1000;//归一化后，所有数值变化都在0到1000之间
  }
  
  for(i = 0;i < 4;i++)           
  {
    ad[i] = (((int16)ad[i])/10)*10; //将数值中个位数除掉
  }
  
  DirectionError[0] = (float)(ad[0] - ad[1])/(ad[0] + ad[1]);      //水平电感的差比和作为偏差（核心）
  DirectionError[0] = (DirectionError[0]>= 1? 1:DirectionError[0]);	//偏差限幅，实际上是达不到的
  DirectionError[0] = (DirectionError[0]<=-1?-1:DirectionError[0]);
  temp_1 = DirectionError[0]*1000;
 
  DirectionErrorTemp_1[3] = DirectionErrorTemp_1[2];
  DirectionErrorTemp_1[2] = DirectionErrorTemp_1[1];
  DirectionErrorTemp_1[1] = DirectionErrorTemp_1[0];
  DirectionErrorTemp_1[0] = DirectionError[0];
  
  DirectionError_dot[0] = 4*(DirectionErrorTemp_1[0]-DirectionErrorTemp_1[3]);//水平电感的偏差微分
  DirectionError_dot[0] = (DirectionError_dot[0]> 0.7? 0.7:DirectionError_dot[0]);//偏差微分限幅
  DirectionError_dot[0] = (DirectionError_dot[0]<-0.7?-0.7:DirectionError_dot[0]);
  temp_2 = DirectionError_dot[0]*1000;
   
  DirectionError[1] = (float)((ad[2] - ad[3])/(ad[2] + ad[3]));//垂直电感的差比和作为偏差
  DirectionError[1] = (DirectionError[1]>= 1? 1:DirectionError[1]);	//偏差限幅
  DirectionError[1] = (DirectionError[1]<=-1?-1:DirectionError[1]);
  temp_3 = DirectionError[1]*1000;
  
  DirectionErrorTemp_2[3] = DirectionErrorTemp_2[2];
  DirectionErrorTemp_2[2] = DirectionErrorTemp_2[1];
  DirectionErrorTemp_2[1] = DirectionErrorTemp_2[0];
  DirectionErrorTemp_2[0] = DirectionError[1];
  
  DirectionError_dot[1] = 4*(DirectionErrorTemp_2[0]-DirectionErrorTemp_2[3]);//垂直电感的偏差微分
  DirectionError_dot[1] = (DirectionError_dot[1]> 0.7? 0.7:DirectionError_dot[1]);//偏差微分限幅
  DirectionError_dot[1] = (DirectionError_dot[1]<-0.7?-0.7:DirectionError_dot[1]);
  temp_4 = DirectionError_dot[1]*1000;
  
}
void ips_adcprint(void)//需移植
{
  ips200_showstr(150,1,"ad0");
  ips200_showstr(250,1,"ad1");
  ips200_showstr(150,2,"ad2");
  ips200_showstr(250,2,"ad3");
  ips200_showuint16(0,1,(unsigned short int)ad[0]);
  ips200_showuint16(80,1,(unsigned short int)ad[1]);
  ips200_showuint16(0,2,(unsigned short int)ad[2]);
  ips200_showuint16(80,2,(unsigned short int)ad[3]);   
}
void shangweiji()
{
  uint8 shangweiji_ad0[7]="ad0:";
   uint8 shangweiji_ad1[7]="ad1:";
    uint8 shangweiji_ad2[7]="ad2:";
     uint8 shangweiji_ad3[7]="ad3:";
     uint8 kongge[2]="  ";
     shangweiji_ad0[4]=((unsigned short int)ad[0]/100)+48;
     shangweiji_ad0[5]=(((unsigned short int)ad[0]/10)%10)+48;
     shangweiji_ad0[6]=((unsigned short int)ad[0]%10)+48;
     seekfree_wireless_send_buff(shangweiji_ad0,7);
     seekfree_wireless_send_buff(kongge,2);
     
     shangweiji_ad1[4]=((unsigned short int)ad[1]/100)+48;
     shangweiji_ad1[5]=(((unsigned short int)ad[1]/10)%10)+48;
     shangweiji_ad1[6]=((unsigned short int)ad[1]%10)+48;
     seekfree_wireless_send_buff(shangweiji_ad1,7);
     seekfree_wireless_send_buff(kongge,2);
     
     shangweiji_ad2[4]=((unsigned short int)ad[2]/100)+48;
     shangweiji_ad2[5]=(((unsigned short int)ad[2]/10)%10)+48;
     shangweiji_ad2[6]=((unsigned short int)ad[2]%10)+48;
     seekfree_wireless_send_buff(shangweiji_ad2,7);
     seekfree_wireless_send_buff(kongge,2);
     
     shangweiji_ad3[4]=((unsigned short int)ad[3]/100)+48;
     shangweiji_ad3[5]=(((unsigned short int)ad[3]/10)%10)+48;
     shangweiji_ad3[6]=((unsigned short int)ad[3]%10)+48;
     seekfree_wireless_send_buff(shangweiji_ad3,7);
}      
    
    

float abs_f(float k) 
{
  if(k<0)     return -k;
  else        return k;
}

