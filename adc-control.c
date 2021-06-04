/*
 * adc-control.c
 *
 *  Created on: 2021��3��3��
 *      Author: dade
 */
#include "headfile.h"
#include "adc-control.h"

int  y; //������־λ     ���٣������䣩
int  z; //������־λ    ���٣���ֱ�ߣ�
int  i; //������־λ      ������С����ٶ��ߣ���ֱ�ߣ�
int  e; //������־λ      ������ǰ�ļ���

/************����·�ڸ�������*********************/

#define S_MOTOR_PIN   ATOM1_CH1_P33_9
#define turn_right    pwm_duty(S_MOTOR_PIN,850)
#define turn_left    pwm_duty(S_MOTOR_PIN,1150)
#define delay systick_delay_ms(STM0,500)
//***********************************//

int sflag=0;//�ڱ�־
int tflag=0;//�ܱ�־
int oflag=0;//�ܱ�־
int sw;//ʮ��·������
int sn;//ʮ��·����
uint8 Flag_Stop;
float dirControl_P = 400;//300;//600;   //�������P  550
float dirControl_D = 500;//300;//270;   //�������D
float dirControl_P_cr = 1000;   //Բ������P
float dirControl_D_cr = 3000;   //Բ������D
 int adcrecognition=0;//���ʶ��
float DirectionError[3];
//��DirectionError[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ�
//��DirectionError[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ�

float DirectionError_dot[2];   //��DirectionError_dot[0]Ϊһ��ˮƽ��еĲ�Ⱥ�ƫ��΢�֣�
//��DirectionError_dot[1]Ϊһ�Դ�ֱ��еĲ�Ⱥ�ƫ��΢�֣�

float DirectionControlOut;  //����������
float temp_1=0.0;
float temp_2=0.0;
float temp_3=0.0;
float temp_4=0.0;
float temp_5=0.0;

float DirectionErrorTemp_1[4]={0,0,0,0};
float DirectionErrorTemp_2[4]={0,0,0,0};

float  ad[5] = {0,0,0,0,0};
float ad2;
float ad3;
float  ad_temp[5] = {0,0,0,0,0};
float  ad_temp_1[5] = {0,0,0,0,0};
int  pidbiaozhi;
int  dianganhe;
float  dianganbi_1;
float  dianganbi_2;
int     ad_temp_error;
int ad_2_chazhi[4]={0,0,0,0};
float ad_2_error;

int    ad_min[5] = {0,0,0,0,0};
int    ad_max[5] ={2100,2100,2100,2100,2100}; //{435,450,420,435,500};
int16  ValueOfADFilter[4]={0};
int16  ValueOfAD[5]={0};    //��ȡ�ĵ��ֵ

uint8  Flag_Round = 0;  //������־
int    L_huan;//�󻷱�־
int    R_huan;//�һ���־
int    cr_flag1_l;
int    cr_flag1_r;//������־
float    Symbol;
float xin;

int sancha_jin=0;
int sancha_qing=0;
int huandao_panduan=0;
int huandao_zuo=0;
int huandao_you=0;
int huandao_chu=0;

//kalman_struct *kalman_lcw;



void Get_AD(void)
{
    kalman_init(&kalman_lcw,-0.5,50.1111);
  {
  uint16  get_ad[5][4],temple;
  int16  i,j,k;
  int16  ad_sum[4];
  int16  ValueOfADOld[4],ValueOfADNew[4];

  for(i=0;i<4;i++)//����ֲ
  {
      get_ad[0][i] = adc_mean_filter(POWER_ADC1_MOD,POWER_ADC0_Pin,ADC_12BIT,10);      //ad0     B14
       get_ad[1][i] = adc_mean_filter(POWER_ADC3_MOD,POWER_ADC1_Pin,ADC_12BIT,10);      //ad1     A15
       get_ad[2][i] = adc_mean_filter(POWER_ADC2_MOD,POWER_ADC3_Pin,ADC_12BIT,10);      //ad2     B21
       get_ad[3][i] = adc_mean_filter(POWER_ADC4_MOD,POWER_ADC4_Pin,ADC_12BIT,10);
       get_ad[4][i] = adc_mean_filter(POWER_ADC4_MOD,POWER_ADC2_Pin,ADC_12BIT,10);
  }
  ////////////////////////ð������////////////////////////////////
  /*for(i=0;i<=4;i++)
  {
    for(j=0;j<4;j++)
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
  }*/
  for(i=0;i<5;i++)    //���м�����ĺ�
  {
    ad_sum[i] = get_ad[i][1] + get_ad[i][2]+get_ad[i][0]+get_ad[i][3] ;
    ad_temp[i] = ad_sum[i] / 4;
  }
  //�˲�  �ɼ��ݶ�ƽ����ÿ�βɼ����仯40
  for(i=0;i<5;i++)
  {


    ValueOfAD[i] = ((int16)(ad_temp[i]/10)*10); //����ֵ�и�λ������

    /*if(ValueOfADNew[i]>=ValueOfADOld[i])
         ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])>20?(ValueOfADOld[i]+20):ValueOfADNew[i]);
       else
         ValueOfADFilter[i] = ((ValueOfADNew[i]-ValueOfADOld[i])<-20?(ValueOfADOld[i]-20):ValueOfADNew[i]);*/


    ad_temp[i]=ValueOfAD[i];
  }

  for(i = 0; i < 5; i++)
  {
       ad_temp_1[i] = (((int16)ad_temp[i])*10)/10;
           // ad_temp_1[i] = ad_temp[i];

  }




  for(i = 0; i < 5; i++)
  {
    if(ad_temp[i] < ad_min[i])
    {
      ad_temp[i]=ad_min[i];
    }
    if(ad_temp[i]>ad_max[i])
    {
      ad_temp[i]=ad_max[i];
    }
    ad[i] = ((ad_temp[i] - ad_min[i]) / (ad_max[i] - ad_min[i])) *1000;//��һ����������ֵ�仯����0��1000֮��
   //ad2=ad[2];
   //ad3=ad[3];
  }
  dianganhe=ad_temp[0]+ad_temp[1];
  dianganbi_1=dianganhe/5;
  dianganbi_2=dianganhe/5;
  ad_temp_error=ad_temp[0]-ad_temp[1];
  if(ad_temp_error<0)
      {
          ad_temp_error=-ad_temp_error;
      }


    for(i = 0;i < 5;i++)
    {
      ad[i] = (((int16)ad[i])/10)*10; //����ֵ�и�λ������
    }
    ad_2_chazhi[3]=ad_2_chazhi[2];
    ad_2_chazhi[2]=ad_2_chazhi[1];
    ad_2_chazhi[1]=ad_2_chazhi[0];
    ad_2_chazhi[0]=ad[0]+ad[1];
    ad_2_error=ad_2_chazhi[3]-ad_2_chazhi[0];
   // ips200_showuint16(0,8,temp_1);

  for(i = 0;i < 5;i++)
  {
    ad[i] = (((int16)ad[i])/10)*10; //����ֵ�и�λ������
  }
  //for(i = 0;i < 4;i++)
    //{
      //ad[i] = (((int16)ad[i])/10)*10; //����ֵ�и�λ������
    //}


  temp_5= (float)(ad[0] - ad[3])*100.0/(ad[0] + ad[3]);  //ˮƽ��еĲ�Ⱥ���Ϊƫ����ģ�
        DirectionError[2] = (float)((0.7*(ad[0] - ad[3])+0.5*(ad[1] - ad[2]))/((0.7*(ad[0] + ad[3])+0.5*abs_f(ad[1] - ad[2]))*1.0));  //ˮƽ��еĲ�Ⱥ���Ϊƫ����ģ�


        DirectionError[0] = kalman_filter(&kalman_lcw,DirectionError[2]);
        //
        //ips200_showfloat(0,3,DirectionError[0],2,5);

        DirectionError[0] = (DirectionError[0]>= 1? 1:DirectionError[0]); //ƫ���޷���ʵ�����Ǵﲻ����
    DirectionError[0] = (DirectionError[0]<=-1?-1:DirectionError[0]);
    temp_1 = DirectionError[0]*1000;


    DirectionErrorTemp_1[3] = DirectionErrorTemp_1[2];
    DirectionErrorTemp_1[2] = DirectionErrorTemp_1[1];
    DirectionErrorTemp_1[1] = DirectionErrorTemp_1[0];
    DirectionErrorTemp_1[0] = DirectionError[0];

    DirectionError_dot[0] = 5*(DirectionErrorTemp_1[0]-DirectionErrorTemp_1[3]);//ˮƽ��е�ƫ��΢��
    DirectionError_dot[0] = (DirectionError_dot[0]> 0.7? 0.7:DirectionError_dot[0]);//ƫ��΢���޷�
    DirectionError_dot[0] = (DirectionError_dot[0]<-0.7?-0.7:DirectionError_dot[0]);
    temp_2 = DirectionError_dot[0]*1000;

    DirectionError[1] = (float)((ad[1] - ad[2])/(ad[1] + ad[2]));//��ֱ��еĲ�Ⱥ���Ϊƫ��
    DirectionError[1] = (DirectionError[1]>= 1? 1:DirectionError[1]); //ƫ���޷�
    DirectionError[1] = (DirectionError[1]<=-1?-1:DirectionError[1]);
    temp_3 = DirectionError[1]*1000;

    DirectionErrorTemp_2[3] = DirectionErrorTemp_2[2];
    DirectionErrorTemp_2[2] = DirectionErrorTemp_2[1];
    DirectionErrorTemp_2[1] = DirectionErrorTemp_2[0];
    DirectionErrorTemp_2[0] = DirectionError[1];

    DirectionError_dot[1] = 5*(DirectionErrorTemp_2[0]-DirectionErrorTemp_2[3]);//��ֱ��е�ƫ��΢��
    DirectionError_dot[1] = (DirectionError_dot[1]> 0.7? 0.7:DirectionError_dot[1]);//ƫ��΢���޷�
    DirectionError_dot[1] = (DirectionError_dot[1]<-0.7?-0.7:DirectionError_dot[1]);
    temp_4 = DirectionError_dot[1]*1000;
    ////////����
     // #if 1/**************** �����ж� ***************///�����//    by     song
    /*   if (ad[1]+ ad[0]>1000 && ad[1]-ad[0]>120&&L_huan==0)//�һ���־λ
         {
          R_huan = 1;
          if(R_huan == 1)
          { L_huan=0;}
         }
         if (ad[1]+ ad[0]>970 && ad[0]-ad[1]>120&&R_huan==0)//�󻷱�־λ
         {
           L_huan = 1;
           if(L_huan ==1)
          { R_huan=0;}
         }
          // huanjiansu();
        if (ad[2] > 900&&Flag_Round==0)
         {
           cr_flag1 = 1;  //�� �� �� �� Ԫ �� �� ־ λ
         }
        if (cr_flag1 == 1 && ad[2] < 750)
         {
           Flag_Round=1;  //�� �� λ �� �� ־ λ
         }
         if (Flag_Round == 1 && ad[2]> 750)
         {
           cr_flag1 = 0;
          // systick_delay_ms(500);
        //Flag_Round=0; cr_flag1 = 0;
          //ick_delay_ms(5000);
        //_huan =0;
        //_huan =0;
         //ag_Round=0;
         }
       /* if(ad[2]==310&&ad[0]<600&&ad[1]<600&&ad[0]>500&&ad[1]>500)
        {
         L_huan =0;
         R_huan =0;
         Flag_Round=0;
        }
       if(ad[1]-ad[0]>280||ad[1]-ad[0]<-280)
       {
            y=1;z=0;i=0;e=0;

       }
        if(ad[2]>305&&ad[2]<335&&ad[1]-ad[0]>20&&ad[1]-ad[0]<-20)

         { z=1;y=0;i=0;e=0;}

        if  (ad[1]-ad[0]>100&&ad[1]-ad[0]<280||ad[1]-ad[0]<-100&&ad[1]-ad[0]<-280)
               { i=1;y=0;z=0;e=0;}
        if  (ad[2]>370)
        {  i=0;y=0;z=0;e=1;   }


       /*if(x==1&&ad[2]> 360&&ad[2]< 370&&ad[0]-ad[1]<30 && ad[0]-ad[1]>10)//�� �� λ �� �� ־ λ
         {
           cr_flag1 = 2;

         }*/


       /*if ((ad[3] -ad[2])>200 && Flag_Round==0 && ad[3]>800)//���������ĵ�У�
         {
           cr_flag1 = 1;  //�� �� �� �� Ԫ �� �� ־ λ
         }
        if (cr_flag1 == 1 && (ad[3] -ad[2]) >250)
         {
           Flag_Round = 1;   //�� �� λ �� �� ־ λ
         }
       //if(Flag_Round==1 &&(ad[3] >ad[2]))//�� �� λ �� �� ־ λ
        // {
         //  cr_flag1 = 0;
        // }*/
       //  #endif

        }

  }






void ips_adcprint(void)

{ h=gpio_get(P33_4);
    if(h==1)
{
   ips200_showuint16(0,6,(unsigned short int)tflag);
ips200_showuint16(80,6,(unsigned short int)sflag);
ips200_showuint16(160,6,(unsigned short int)oflag);
ips200_showuint16(0,0,(unsigned short int)PWM);

    //ips200_showuint16(160,3,(unsigned short int)ad4[1]);

    ips200_showstr(0,1,"0:");
    ips200_showuint16(17,1,(unsigned short int)ad[0]);
    ips200_showstr(80,1,"1:");
    ips200_showuint16(97,1,(unsigned short int)ad[1]);
    ips200_showstr(160,1,"2:");
    ips200_showuint16(177,1,(unsigned short int)ad[2]);
    ips200_showstr(240,1,"3:");
    ips200_showuint16(257,1,(unsigned short int)ad[3]);
    ips200_showstr(240,0,"4:");
       ips200_showuint16(257,0,(unsigned short int)ad[4]);

    ips200_showuint16(160,2,(unsigned short int)L_huan);
     ips200_showuint16(0,2,(unsigned short int)cr_flag1_l);
    ips200_showuint16(80,2,(unsigned short int)Flag_Round);
    ips200_showuint16(220,2,(unsigned short int)R_huan);
    ips200_showuint16(220,3,(unsigned short int)cr_flag1_r);
    ips200_showuint16(160,10,(unsigned short int)tflag);
    ips200_showuint16(160,11,(unsigned short int)sflag);

    ips200_showuint16(0,5,(unsigned short int)ad_temp[0]);
    ips200_showuint16(160,5,(unsigned short int)ad_temp[1]);
    ips200_showuint16(80,5,(unsigned short int)ad_temp[2]);

    ips200_showuint16(0,7,(unsigned short int)sw);
    ips200_showuint16(80,7,(unsigned short int)sn);
    ips200_showuint16(0,10,(unsigned short int)(DirectionError[0]*100));

    ips200_showfloat(0,11,Filte_Angle,3,3);
}
#if 1

    if (ad[4]> 740&&Flag_Round==0 )
                {
                  cr_flag1_r = 1;  //�� �� �� �� Ԫ �� �� ־ λ
                }
    if ( ad[2]-ad[1]>150 &&cr_flag1_r==1&& L_huan ==0&& R_huan ==0)//�һ���־λ  1000       120
         {
          R_huan = 1;
          if(R_huan == 1)
          { L_huan=0;}
         }
    if (ad[4]> 740&&Flag_Round==0 )
                    {
                      cr_flag1_l = 1;  //�� �� �� �� Ԫ �� �� ־ λ
                    }
        if ( ad[1]-ad[2]>150 &&cr_flag1_l==1&& L_huan ==0&& R_huan ==0)//�һ���־λ  1000       120
             {
              L_huan = 1;
              if(L_huan == 1)
              { R_huan=0;}
             }
    /*
        if (ad[1]-ad[2]>100 && cr_flag1_l==0&& ad[4]>600&&ad[4]<700)//�󻷱�־λ  970     120
         {
          L_huan = 1;
           if(L_huan ==1)
          { R_huan=0;}
         }
         if (ad[4]> 670&& L_huan == 1 &&ad[1]-ad[2]>210)
                     {
                       cr_flag1_l = 1;  //�� �� �� �� Ԫ �� �� ־ λ
                     }
          // huanjiansu();
*/
        if ((cr_flag1_l==1||cr_flag1_r==1) &&  ((ad[0]<200&&R_huan == 1)||(ad[3]<200&&L_huan == 1)))
         {
           Flag_Round=1;  //�� �� λ �� �� ־ λ
         }
         if (Flag_Round == 1 && ((ad[1]>500&&R_huan == 1)||(ad[2]>500&&L_huan == 1)))
         {
             cr_flag1_l=0;cr_flag1_r=0;
         }
           if (cr_flag1_l==0&&cr_flag1_r==0 && ad[1]+ad[2]<100)
           {
             Flag_Round=0;
             R_huan =0;
             L_huan = 0;
             //�� �� λ �� �� ־ λ
           }
          // systick_delay_ms(500);
        //Flag_Round=0; cr_flag1 = 0;
            //stick_delay_ms(5000);
        //_huan =0;
        //_huan =0;
         //ag_Round=0;







         /*********************************************����·��******************************by song*******************************************************/

         if(tflag==0 && sflag==0&&ad[4]<300&&ad[1]>50&&ad[2]>50&&ad[1]<200&&ad[2]<200&&Flag_Round==0&&ad[0]>170&&ad[3]>170) //��ȥʱ�������
         {
             //turn_right;
             tflag = 1;

         }
         if(tflag==1 && sflag==0 && ad[0]<100)//��ȥ�ö������
         {

             sflag=1;
         }
         if(tflag==1 && sflag==1 && ad[1]+ ad[2]>400)//��ȥ������
                 {


             oflag=1;
             sflag=2;

                 }

         if(tflag==1 && sflag==2 && oflag==1 && ad[2]+ad[1]<50 )  //��ȥ��ֱ����ʱ�����־λ
         {

             tflag = 2;
             oflag=2;
             sflag=0;

         }
         if( tflag==2 && sflag==0 && oflag==2 &&ad[4]<300&&ad[1]>50&&ad[2]>50&&ad[1]<200&&ad[2]<200&&Flag_Round==0&&ad[0]>170&&ad[3]>170)//��ȥ������
             {

             tflag = 3;
             }

         if(tflag==3 && sflag==0 && oflag==2 && ad[3]<100 )//��ȥ�ö������
                 {

                     sflag=1;
                 }
          if(tflag==3 && sflag==1 && oflag==2 && ad[1]+ ad[2]>400)//��ȥ������
                         {


              sflag=2;
              oflag=3;

                         }

                 if(tflag==3 && sflag==2 && oflag==3  &&  ad[2]+ad[1]<50)  //��ȥ��ֱ����ʱ�����־λ
                 {

                     tflag = 4;
                     oflag=4;
                     sflag=0;

                 }

/******************************************************ʮ��·��************************************/







         /*if(ad[2]>450 && tflag==3 && sflag==1)  //ad[2]<200 && ad[2]>160 && tflag==3 && sflag==1&&ad[0]+ad[1]+ad[2]>1000 && ad[0]+ad[1]+ad[2]<1650
             {
                 //turn_right;
             tflag = 4;
                 sflag=0;
             }*/




       /* if(ad[2]==310&&ad[0]<600&&ad[1]<600&&ad[0]>500&&ad[1]>500)
        {
         L_huan =0;
         R_huan =0;
         Flag_Round=0;
        }
       if(ad[1]-ad[0]>280||ad[1]-ad[0]<-280)
       {
            y=1;z=0;i=0;e=0;

       }
        if(ad[2]>305&&ad[2]<335&&ad[1]-ad[0]>20&&ad[1]-ad[0]<-20)

         { z=1;y=0;i=0;e=0;}

        if  (ad[1]-ad[0]>100&&ad[1]-ad[0]<280||ad[1]-ad[0]<-100&&ad[1]-ad[0]<-280)
               { i=1;y=0;z=0;e=0;}
        if  (ad[2]>370)
        {  i=0;y=0;z=0;e=1;   }


       /*if(x==1&&ad[2]> 360&&ad[2]< 370&&ad[0]-ad[1]<30 && ad[0]-ad[1]>10)//�� �� λ �� �� ־ λ
         {
           cr_flag1 = 2;

         }*/


       /*if ((ad[3] -ad[2])>200 && Flag_Round==0 && ad[3]>800)//���������ĵ�У�
         {
           cr_flag1 = 1;  //�� �� �� �� Ԫ �� �� ־ λ
         }
        if (cr_flag1 == 1 && (ad[3] -ad[2]) >250)
         {
           Flag_Round = 1;   //�� �� λ �� �� ־ λ
         }
       //if(Flag_Round==1 &&(ad[3] >ad[2]))//�� �� λ �� �� ־ λ
        // {
         //  cr_flag1 = 0;
        // }*/
         #endif

        }
      /* void ips_adcprint(void)//IPS��ʾ�ĸ����ֵ  ������־λ
       {
           ips200_showstr(0,2,"ad0");
         ips200_showstr(80,2,"ad2");
         ips200_showstr(160,2,"ad1");

         ips200_showuint16(0,3,(unsigned short int)ad[0]);
         ips200_showuint16(160,3,(unsigned short int)ad[1]);
         ips200_showuint16(80,3,(unsigned short int)ad[2]);

           ips200_showstr(0,4,"Crflag1");
         ips200_showstr(80,4,"FlagRound");
         ips200_showstr(160,4,"LHuan");
         ips200_showstr(240,4,"RHuan");

         ips200_showuint16(160,5,(unsigned short int)L_huan);
          ips200_showuint16(0,5,(unsigned short int)cr_flag1);
         ips200_showuint16(80,5,(unsigned short int)Flag_Round);
         ips200_showuint16(240,5,(unsigned short int)R_huan);

             ips200_showstr(0,8,"ad1+ad2");
           //  ips200_showint8(160,10,x);
         ips200_showstr(80,8,"ad1-ad2");
         ips200_showint16(0,9,(unsigned short int)(ad[1]+ ad[0]));
         ips200_showint16(80,9,(unsigned short int)(ad[1]- ad[0]));
       }*/




/************************����·��****************************/




       void shangweiji(void)//��λ����ʾ�ĸ����ֵ
       {
         uint8 txee0[7]="ad0:";
         uint8 txee1[7]="ad1:";
         uint8 txee2[7]="ad2:";
         uint8 txee3[7]="ad3:";
         uint8 txee5[2]="  ";
         txee0[4]=((unsigned short int)ad[0]/100)+48;
         txee0[5]=(((unsigned short int)ad[0]/10)%10)+48;
         txee0[6]=((unsigned short int)ad[0]%10)+48;
         seekfree_wireless_send_buff(txee0,7);
       seekfree_wireless_send_buff(txee5,2);

         txee1[4]=((unsigned short int)ad[1]/100)+48;
         txee1[5]=(((unsigned short int)ad[1]/10)%10)+48;
         txee1[6]=((unsigned short int)ad[1]%10)+48;
         seekfree_wireless_send_buff(txee1,7);
       seekfree_wireless_send_buff(txee5,2);


         txee2[4]=((unsigned short int)ad[2]/100)+48;
         txee2[5]=(((unsigned short int)ad[2]/10)%10)+48;
         txee2[6]=((unsigned short int)ad[2]%10)+48;
         seekfree_wireless_send_buff(txee2,7);
       seekfree_wireless_send_buff(txee5,2);

        txee3[4]=((unsigned short int)ad[3]/100)+48;
         txee3[5]=(((unsigned short int)ad[3]/10)%10)+48;
         txee3[6]=((unsigned short int)ad[3]%10)+48;
         seekfree_wireless_send_buff(txee3,7);
       seekfree_wireless_send_buff(txee5,2);
       wireless_uart_callback();
       }

       float abs_f(float k)
       {
         if(k<0)     return -k;
         else        return k;
       }

float Cha_BI_He(float Date_1,float Date_2,int X)
       {
           float Cha = 0;
           float He = 0;
           float Result;
           Cha = Date_1-Date_2;
           He =Date_1+Date_2;
           Result = (Cha/He)*X;
           return Result;

       }


