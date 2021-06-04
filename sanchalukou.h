#ifndef CODE___SANCHALUKOU_H__
#define CODE___SANCHALUKOU_H__
#include "include.h"
typedef struct {
    float x;  // state
    float A;  // x(n)=A*x(n-1)+u(n),u(n)~N(0,q)
    float H;  // z(n)=H*x(n)+w(n),w(n)~N(0,r)
    float q;  // process(predict) noise convariance 协方差
    float r;  // measure noise convariance
    float p;  // estimated error convariance 估计误差协方差
    float gain;
}kalman_struct;
extern kalman_struct kalman_lcw;

extern float abs_f(float k);
extern float  ad[5];


extern void turn_right_out_san_cha(void);//右出三叉
extern void turn_left_in_san_cha(void);//左进三叉
extern void turn_left_out_san_cha(void);//左出三叉

extern void san_cha(void);

extern void kalman_init(kalman_struct* kalman_lcw, float init_x, float init_p);
extern float kalman_filter(kalman_struct* kalman_lcw, float z_measure);
extern void EveryInit(void);


struct acc
{
        float x;
        float y;
        float z;

};

 struct gyro
{
        float x;
        float y;
        float z;
};

typedef struct
{
        struct acc acc;
        struct gyro gyro;
}ICM_rain;

extern ICM_rain ICM_Start1;
extern ICM_rain ICM_Offset;
extern ICM_rain ICM_Real;


/********函数声明*********/
void Complementary_Filter(float acc_angle,float gyro_angle);
void ACC_GYRO_Offset_Get(void);
void Data_Filter(void); // 数据滤波
void KalmanFilter(float ACC_Angle);
void Get_Attitude(void);

/*********全局变量声明********/

extern  float Filte_Angle,angleAy; //滤波后的角度、反正切求的角度
extern float  Angle_Zero ;//角度零偏值（这个就是车子平衡的实际角度）
extern float  AngleIntegral;






#endif
