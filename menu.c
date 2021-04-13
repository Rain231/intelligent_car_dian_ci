
/*
 * Dream-Seekers-menu.c
 *
 *  Created on: 2020年4月17日
 *      Author: 韩金雨
 *     Version: V1.0
 *        Core: TC264D
 *
 *   	  Name: 多级菜单驱动
 *		 Brief:
 *	      Note:
 */

#include "headfile.h"
void IPS_TuoLuoYi_DISPLAY()
{
	
	
}

void IPS_STEER_DISPLAY()
{
	ips200_showstr(0,0,"duoji");
    ips200_showuint16(0,1,(unsigned short int)PWM);//显示舵机PWM
	
}
void IPS_ADC_DISPLAY()
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
	
}
void IPS_V_DISPLAY()
{
    ips200_showstr(0,6,"FB_L_dj");
    ips200_showstr(80,6,"FB_R_dj");

    ips200_showint16(0,7,-FeedBack_L);
    ips200_showint16(80,7,-FeedBack_R);	
	
}
void MenuReflash (void)
{
  	static unsigned char 	s=0;
	if(!s)			{s=1;	IPS_TuoLuoYi_DISPLAY();IPS_STEER_DISPLAY();IPS_ADC_DISPLAY();IPS_V_DISPLAY();};		//第一次进入（如果需要）启动屏幕和按键，并且载入界面

}