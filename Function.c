#include "Function.h"
/**********鐩寸珛PID鍙傛暟******/
PID  AngleVelPID; //瑙掗�熷害鐜疨ID鍌ㄥ瓨锛堝彧鏄偍瀛樼殑涓棿閲忥紝鏃犻渶鍏冲績锛�
float AngleVel_Pid[4] = {0.15, 0.008, 0.001, 1000};		// 瑙掗�熷害鐜疨ID

PID  AnglePID ;//瑙掑害鐜疨ID鍌ㄥ瓨锛堝彧鏄偍瀛樼殑涓棿閲忥紝鏃犻渶鍏冲績锛�
float Angle_Pid[4]   = {0.5, 0, 0.009, 500};		// 瑙掑害鐜疨ID

/********閫熷害PID鍙傛暟********/
PID SpeedPID;//閫熷害PID鍌ㄥ瓨锛堝彧鏄偍瀛樼殑涓棿閲忥紝鏃犻渶鍏冲績锛�

float Speed_Pid[4]  = {3.8,0.001,0, 500};		// 閫熷害鐜疨ID
/*******杞悜PID************/
PID DirectVelPID;//杞悜鍐呯幆PID鍌ㄥ瓨锛堝彧鏄偍瀛樼殑涓棿閲忥紝鏃犻渶鍏冲績锛�
float DirectVel_Pid[4]  = {0.006,0, 0.01, 100};	// 杞悜鍐呯幆PID 浣嶇疆	0.017	0.02
PID TurnPID;//杞悜PID鍌ㄥ瓨锛堝彧鏄偍瀛樼殑涓棿閲忥紝鏃犻渶鍏冲績锛�
float Turn_Pid[][4]  = { {0.005,1,0,1000},        // 鏈�鍚庝竴椤逛负绉垎闄愬箙  涓茬骇鎺у埗鍙傛暟
                          {30,3,0,0},            //鏅�氳浆鍚戝弬鏁�
                          {5,2.5,0,0}, };

uint8 Turn_Suquence = 0;//杞悜PID閫夋嫨


// PID鍙傛暟鍒濆鍖�
void PID_Parameter_Init(PID *sptr)
{
	sptr->SumError  = 0;
	sptr->LastError = 0;	//Error[-1]
	sptr->PrevError = 0;	//Error[-2]	
	sptr->LastData  = 0;
}


// ********************浣嶇疆寮忓姩鎬丳ID鎺у埗************************************
/*
鍑芥暟锛歩nt32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
鍔熻兘锛氫綅缃紡鍔ㄦ�丳ID鎺у埗
鍙傛暟锛�
PID *sprt锛�      缁撴瀯浣撴寚閽�
float *PID锛�     PID鏁扮粍  锛堥�氳繃鏁扮粍瀹氫箟PID鍊硷級
int32 NowPiont锛� 褰撳墠鍊�  锛堝彲浣跨敤缁撴瀯浣撳畾涔夊彉閲忥級
int32 SetPoint锛� 璁惧畾鐩爣鍊�   杞悜鎺у埗涓瀹氬�间负0銆�

璇存槑锛�  璇ュ嚱鏁板弬鑰冨叾浠栫▼搴忋�傚姩鎬佹帶鍒朵竴鑸敤浜庤浆鍚戞帶鍒�
杩斿洖鍊硷細 int32 Realize
eg锛歊adius = PlacePID_Control(&Turn_PID, Turn[Fres], Difference, 0);// 鍔ㄦ�丳ID鎺у埗杞悜
鏃ユ湡锛� 2鏈�1鏃�
浣滆�咃細  閭ｄ釜娣峰瓙     */
// 浣嶇疆寮忓姩鎬丳ID鎺у埗
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint)
{
	//瀹氫箟涓哄瘎瀛樺櫒鍙橀噺锛屽彧鑳界敤浜庢暣鍨嬪拰瀛楃鍨嬪彉閲忥紝鎻愰珮杩愮畻閫熷害
	int32 iError,	//褰撳墠璇樊
		  Actual;	//鏈�鍚庡緱鍑虹殑瀹為檯杈撳嚭鍊�
	float Kp;		//鍔ㄦ�丳
	iError = SetPoint - NowPiont;	//璁＄畻褰撳墠璇樊
	sprt->SumError += iError*0.01;
	if (sprt->SumError >= PID[KT])
	{
		sprt->SumError = PID[KT];
	}
	else if (sprt->SumError <=-PID[KT])
	{
		sprt->SumError = -PID[KT];
	}
          
	Kp = 1.0 * (iError*iError)/PID[KP] + PID[KI];	//P鍊间笌宸�兼垚浜屾鍑芥暟鍏崇郴锛屾澶凱鍜孖涓嶆槸PID鍙傛暟锛岃�屾槸鍔ㄦ�丳ID鍙傛暟锛岃娉ㄦ剰锛侊紒锛�
	
	Actual = Kp * iError
		   + PID[KD] * ((0.8*iError + 0.2*sprt->LastError) - sprt->LastError);//鍙敤PD
	sprt->LastError = iError;		//鏇存柊涓婃璇樊

	//Actual += sprt->SumError*0.1;
	//Actual = limit(Actual, 300); //闄愬箙

	return Actual;
}

//************************* 浣嶇疆寮廝ID鎺у埗*****锛堥�熷害PID锛�*********************
/*
鍑芥暟锛歩nt32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
鍔熻兘锛氫綅缃紡PID鎺у埗
鍙傛暟锛�
PID *sprt锛�      缁撴瀯浣撴寚閽�
float *PID锛�     PID鏁扮粍  锛堥�氳繃鏁扮粍瀹氫箟PID鍊硷級
int32 NowData  褰撳墠鍊�  锛堝彲浣跨敤缁撴瀯浣撳畾涔夊彉閲忥級
int32 Point    璁惧畾鐩爣鍊�  锛堝彲浣跨敤缁撴瀯浣撳畾涔夊彉閲忥級

璇存槑锛�  璇ュ嚱鏁板弬鑰冨叾浠栫▼搴忋��
杩斿洖鍊硷細 int32 Realize
eg锛歍ar_Ang_Vel.Y = PID_Realize(&Angle_PID, Angle, (int32)(Attitude_Angle.Y*100), (int32)Target_Angle.Y);	// 缁撴灉涓烘斁澶�10鍊嶇殑鐩爣瑙掗�熷害
鏃ユ湡锛� 2鏈�1鏃�
浣滆�咃細 閭ｄ釜娣峰瓙     */
  ////////////
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//褰撳墠璇樊锛屽畾涔変负瀵勫瓨鍣ㄥ彉閲忥紝鍙兘鐢ㄤ簬鏁村瀷鍜屽瓧绗﹀瀷鍙橀噺锛屾彁楂樿繍绠楅�熷害
	int32 iError;	// 褰撳墠璇樊
	float	 Realize;	// 鏈�鍚庡緱鍑虹殑瀹為檯澧為噺

	iError = Point - NowData;	// 璁＄畻褰撳墠璇樊      璁惧畾鍑忓綋鍓�
	sptr->SumError += PID[KI] * iError;	// 璇樊绉垎
	sptr->SumError = limit1(sptr->SumError, PID[KT]);//绉垎闄愬箙

	Realize = PID[KP] * iError
			+ sptr->SumError
			+ PID[KD] * (iError - sptr->LastError);     //P  I   D  鐩稿姞
	sptr->PrevError = sptr->LastError;	// 鏇存柊鍓嶆璇樊
	sptr->LastError = iError;		  	// 鏇存柊涓婃璇樊
	sptr->LastData  = NowData;			// 鏇存柊涓婃鏁版嵁    娌＄敤 */

	return Realize;	// 杩斿洖瀹為檯鍊�
} 

//-------------------------------------------------------------------------//
//************************澧為噺寮廝ID鐢垫満鎺у埗*********************
/*
鍑芥暟锛歩nt32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
鍔熻兘锛� 澧為噺寮廝ID鐢垫満鎺у埗
鍙傛暟锛�
PID *sprt锛�      缁撴瀯浣撴寚閽�
float *PID锛�     PID鏁扮粍  锛堥�氳繃鏁扮粍瀹氫箟PID鍊硷級
int32 NowData    褰撳墠鍊�  锛堝彲浣跨敤缁撴瀯浣撳畾涔夊彉閲忥級
int32 Point      璁惧畾鐩爣鍊�  锛堝彲浣跨敤缁撴瀯浣撳畾涔夊彉閲忥級

璇存槑锛�  璇ュ嚱鏁板弬鑰冨叾浠栫▼搴忋��
杩斿洖鍊硷細 int32 Increase
eg锛歍heory_Duty += PID_Increase(&Ang_Vel_PID, Ang_Vel, (int32)(GYRO_Real.Y*10), (int32)(Tar_Ang_Vel.Y));	// 璁＄畻鐩寸珛PWM
鏃ユ湡锛� 2鏈�1鏃�
浣滆�咃細 閭ｄ釜娣峰瓙     */
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point)
{
	//褰撳墠璇樊锛屽畾涔変负瀵勫瓨鍣ㄥ彉閲忥紝鍙兘鐢ㄤ簬鏁村瀷鍜屽瓧绗﹀瀷鍙橀噺锛屾彁楂樿繍绠楅�熷害
	int32 iError,	//褰撳墠璇樊
		Increase;	//鏈�鍚庡緱鍑虹殑瀹為檯澧為噺

	iError = Point - NowData;	// 璁＄畻褰撳墠璇樊

	Increase =  PID[KP] * (iError - sptr->LastError)
			  + PID[KI] * iError
			  + PID[KD] * (iError - 2 * sptr->LastError + sptr->PrevError);
	
	sptr->PrevError = sptr->LastError;	// 鏇存柊鍓嶆璇樊
	sptr->LastError = iError;		  	// 鏇存柊涓婃璇樊
	sptr->LastData  = NowData;			// 鏇存柊涓婃鏁版嵁
	
	return Increase;	// 杩斿洖澧為噺
}

//*************************************************************************
/****************闄愬箙****************
//x鏄檺骞呭璞�
//y鏄檺骞呰寖鍥�
//鏈夋璐�


*/
int16 limit1(int16 x, int y)
{
    if(x>y)             return y;
    else if(x<-y)       return -y;
    else                return x;
}

 /******** 闄愬箙淇濇姢 *********/
int32 range_protect(int32 duty, int32 min, int32 max)//闄愬箙淇濇姢
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}

/****************姹傜粷瀵瑰��********/
int  myabs1(int dat)
{
    if(dat>=0)  return dat;
    else        return -dat;
}

/***********寤舵椂鍑芥暟************/
void delay(long t)
{  
	int i;
    while(t--)
   for(i=1000;i>0;i--);
}

/**********涓�闃朵綆閫氭护娉�**********/
#define  a   0.1
/*
value 涓婃婊ゆ尝鍚庣殑鍊�
new_value 鏂扮殑閲囨牱鍊�
out_value 鏈杈撳嚭
婊ゆ尝绯绘暟瓒婂皬锛屾护娉㈢粨鏋滆秺骞崇ǔ锛屼絾鏄伒鏁忓害瓒婁綆銆傛护娉㈢郴鏁拌秺澶э紝鐏垫晱搴﹁秺楂橈紝浣嗘槸婊ゆ尝缁撴灉瓒婁笉绋冲畾
*/
int16  filter(int16 new_value,int16 value)
{
  int16 out_value=0;
  out_value = (1-a)*new_value+a*value ;
return   out_value;
}
/******************涓績鍋忓樊婊ゆ尝***********************
鍑芥暟锛� float  Turn_Out_Filter(float turn_out)
鍙傛暟锛�  鏃�
璇存槑锛�  鏃�
杩斿洖鍊硷細鏃�
鏃ユ湡锛� 11鏈�28鏃�
鏈�鍚庝慨鏀规椂闂达細2019-4-19
浣滆�咃細  閭ｄ釜娣峰瓙     */
int16  Turn_Out_Filter(float turn_out)        
{
  int16 Turn_Out_Filtered;  
  static float Pre1_Error[4]; 
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=(int16)(Pre1_Error[0]*0.5+Pre1_Error[1]*0.3+Pre1_Error[2]*0.1+Pre1_Error[3]*0.1);
  return Turn_Out_Filtered;
} 
/******************鍔犳潈婊ゆ尝***********************
鍑芥暟锛� int16 Weights_Of_Filter(float Date,float value_1,float value_2,float value_3,float value_4)
鍙傛暟锛�  float Date  //瑕佹护娉㈢殑鍊�   float value_1,float value_2,float value_3,float value_4//婊ゆ尝鐨勬潈閲�
璇存槑锛�  鏃�
杩斿洖鍊硷細鏃�
鏃ユ湡锛�2019-5-9
鏈�鍚庝慨鏀规椂闂达細2019-5-9
浣滆�咃細 娣峰瓙     */
float Weights_Of_Filter(float Date,float value_1,float value_2,float value_3)
{
 float Filter_Out;  
  static float Error[4]; 
  Error[3]=Error[2];
  Error[2]=Error[1];
  Error[1]=Error[0];
  Error[0]=Date; 
 Filter_Out=(Error[0]*value_1+Error[1]*value_2+Error[2]*value_3);
  return Filter_Out;
}



/*****************差比积***********************
函数： float Cha_BI_Ji(float date_1,float date_2,int16 x)
参数：  float date_1--第一个数据  float date_2--第二个数据  float x-所求结果放大的倍数
说明：  无
返回值：无
日期：2019-5-10
最后修改时间：2019-5-10
作者：  过敏     */
float Cha_BI_Ji(float date_1,float date_2,int16 x)
{
  float cha=0;
  float ji=0;
  float resault;
  cha =  date_1 - date_2;   //宸�
  ji =   date_1 + date_2;   //鍜�
  resault = (cha/ji)*x ; //宸瘮鍜�
  
  return   resault;
}
/*****************宸瘮绉�***********************
鍑芥暟锛� int16 Cubic_Function(int16 DATE,float A,float B)
鍙傛暟锛�  int16 DATE--鍘熷鏁版嵁  float A--涓夋绯绘暟  float B-涓�娆＄郴鏁�
璇存槑锛�  鏃�
杩斿洖鍊硷細鏃�
鏃ユ湡锛�2019-5-30
鏈�鍚庝慨鏀规椂闂达細2019-5-30
浣滆�咃細  杩囨晱     */
 int16 Cubic_Function(int16 DATE,float A,float B)
{
  int16 Final_Date;
  Final_Date = (DATE*DATE*DATE)*A+DATE*B;
  return   Final_Date;
}

////////鍘婚櫎鏋佸�兼眰骞冲潎
int16 I_Median_Average_Filter(int16 *DATE)
{
    uint8 i;
    int16 max,min;  //瀹氫箟鏋佸�间互鍙婂拰
    int16 sum = 0;

    max = DATE[0];
    min = DATE[0];

    for(i=0;i<sizeof(DATE);i++)
    {
        if(max<DATE[i])max = DATE[i];
        if(min>DATE[i])min = DATE[i];
        sum += DATE[i];
    }

    sum =(sum-max-min)/(sizeof(DATE)-2);    //>>3
    return sum;
}


