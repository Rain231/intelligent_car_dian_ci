#ifndef _Function_h
#define _Function_h


//#include "common.h"
#include "include.h"
//PID
//�⼸�������������±�ţ����㿴
#define KP 0
#define KI 1
#define KD 2
#define KT 3   //�����޷���
//����


typedef struct PID   //����PID�����������
{
	float SumError;	//����ۼ�	
	int32 LastError;	// �ϴ����
	int32 PrevError;	// Ԥ�����	
	int32 LastData;	// �ϴ�����
} PID;

/********PID�м��������************/
extern PID  AngleVelPID; //���ٶȻ�PID���棨ֻ�Ǵ�����м�����������ģ�
extern PID  AnglePID ;//�ǶȻ�PID���棨ֻ�Ǵ�����м�����������ģ�
extern PID SpeedPID;//�ٶ�PID���棨ֻ�Ǵ�����м�����������ģ�
extern PID DirectVelPID;//ת���ڻ�PID���棨ֻ�Ǵ�����м�����������ģ�
extern PID TurnPID;//ת��PID���棨ֻ�Ǵ�����м�����������ģ�

/********PID���������ⲿ����******/
extern float AngleVel_Pid[4];
extern float Angle_Pid[4];
extern float Speed_Pid[4];		// �ٶȻ�PID
extern float DirectVel_Pid[4];	// ת���ڻ�PID λ��	
extern float Turn_Pid[][4];
extern uint8 Turn_Suquence ;//ת��PIDѡ��



//*******************��������***********************

// PID������ʼ��
void PID_Parameter_Init(PID *sptr);
int16 limit1(int16 x, int y);                //�޷�

/**************�޷������˲�һ��ֵ*****/
//#define  limit_ab( x,  min,  max)  ( (x<min) ? min : ( (x>max) ? max : x ))
int32 range_protect(int32 duty, int32 min, int32 max);//�޷�����	
void  delay(long t);                         //��ʱ

int   myabs1(int dat);                        //�����ֵ


int16  filter(int16 new_value,int16 value) ;     //һ�׵�ͨ�˲�

int16  Turn_Out_Filter(float turn_out)     ;         //��Ȩƽ���˲�

float Weights_Of_Filter(float Date,float value_1,float value_2,float value_3);//��Ȩ�˲�  



float Cha_BI_Ji(float date_1,float date_2,int16 x);  //��������Ȼ�

int16 Cubic_Function(int16 DATE,float A,float B);   //��һ�����������η�

// PID������ʼ��
void PID_Parameter_Init(PID *sptr);


// λ��ʽ��̬PID����
int32 PlacePID_Control(PID *sprt, float *PID, int32 NowPiont, int32 SetPoint);

// λ��ʽPID����
int32 PID_Realize(PID *sptr, float *PID, int32 NowData, int32 Point);

// ����ʽPID����
int32 PID_Increase(PID *sptr, float *PID, int32 NowData, int32 Point);
//ȥ��ֵ��ƽ��
int16 I_Median_Average_Filter(int16 *DATE);


#endif
