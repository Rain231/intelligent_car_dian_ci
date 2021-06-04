/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/


#include "Standing.h"

extern uint16  K_P;
extern uint16 K_D ;
uint16  K_P=400;
uint16 K_D=50;



/****************************��ֲ**************
ʹ�������ƴ��������ݣ�����
    uart_putchar(USART_1,0xA5);                             // ����1����0xA5
**********************************************/
void Data_Send(UARTN_enum uratn,signed short int *pst)
{
  unsigned char _cnt=0; unsigned char sum =0;
    unsigned char data_to_send[23] = {0};         //���ͻ���
    unsigned char i;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //��8λ
    data_to_send[_cnt++]=(unsigned char)pst[0];  //��8λ
    data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[1];
    data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[2];
    data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[3];
    data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[4];
    data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[5];
    /*data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[6];
    data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[7];
    data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[8];*/


    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

        for(i=0;i<_cnt;i++)
                uart_putchar(uratn,data_to_send[i]);

}
void datasend()
{
   short send_data[6];

   send_data[0]=Motor_L; ////////MPU_Treated.acc.x
   //send_data[1]=PWM;    //////////////////    MpuStart.gyro.x   Angle
   //send_data[2]=ad[0];//  ////////
   //send_data[3]=ad[1];//
   //send_data[4]=ad[2];
   //send_data[5]=ad[3];
   Data_Send(UART_2,send_data);

}





