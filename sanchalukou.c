
/*
 * adc-control.c
 *
 *  Created on: 2021/4/29
 *      Author: after rain
 *      Kalman filter
 */

//#include "adc-control.h"

#include "include.h"

kalman_struct kalman_lcw;


/**
 *kalman_init - 卡尔曼滤波器初始化
 *@kalman_lcw：卡尔曼滤波器结构体
 *@init_x：待测量的初始值
 *@init_p：后验状态估计值误差的方差的初始值
 */
void kalman_init(kalman_struct* kalman_lcw, float init_x, float init_p)
{
    kalman_lcw->x = init_x;//待测量的初始值，如有中值一般设成中值（如陀螺仪）
    kalman_lcw->p = init_p;//后验状态估计值误差的方差的初始值
    kalman_lcw->A = 1;
    kalman_lcw->H = 1;
    kalman_lcw->q = 10e-2;//10e-6;//2e2;////predict noise convariance 预测（过程）噪声方差 实验发现修改这个值会影响收敛速率
    kalman_lcw->r = 3;//10e-5;//测量（观测）噪声方差。以陀螺仪为例，测试方法是：  5e2
    //保持陀螺仪不动，统计一段时间内的陀螺仪输出数据。数据会近似正态分布，
    //按3σ原则，取正态分布的(3σ)^2作为r的初始化值
}

/**
 *kalman_filter - 卡尔曼滤波器
 *@kalman_lcw:卡尔曼结构体
 *@measure；测量值
 *返回滤波后的值
 */
float kalman_filter(kalman_struct* kalman_lcw, float measure)
{
    /* Predict */
    //kalman_lcw->x = kalman_lcw->A * kalman_lcw->x;
   // kalman_lcw->p = kalman_lcw->A * kalman_lcw->A * kalman_lcw->p + kalman_lcw->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    kalman_lcw->gain = kalman_lcw->p / (kalman_lcw->p  + kalman_lcw->r);
    kalman_lcw->x = kalman_lcw->x + kalman_lcw->gain * (measure -  kalman_lcw->x);
    kalman_lcw->p = (1 - kalman_lcw->gain ) * kalman_lcw->p;

    return kalman_lcw->x;
}

void EveryInit(void)
{
        gtm_pwm_init(S_MOTOR_PIN,50,650);//舵机初始化
        gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);//编码器初始化
        icm20602_init_spi();//陀螺仪初始化
        ips200_init();//IPS屏幕初始化
        seekfree_wireless_init();//无线转串口初始化
        gtm_pwm_init(MOTOR_A,17000,0);//电机初始化
        gtm_pwm_init(MOTOR_B,17000,0);
        //ACC_GYRO_Offset_Get();
        adc_init(ADC_0, POWER_ADC0_Pin);//电感初始化
        adc_init(ADC_0, POWER_ADC1_Pin);
        adc_init(ADC_0, POWER_ADC2_Pin);
        adc_init(ADC_0, POWER_ADC3_Pin);
        adc_init(ADC_0, POWER_ADC4_Pin);
}
//#include "gyroRain.h"

#define   CONSTANT      2     //补偿系数  减小要适当增大D  一般不需改了，1到4之间取个中值
#define   DT          0.0019//积分系数      决定了跟踪速度  跟踪慢了就调大，跟踪过冲了就调小！！！


float Filte_Angle=0.0,angleAy=0.0;//滤波后的角度、反正切求出的角度
float  Angle_Zero = 0.0;//角度零偏值（这个就是车子平衡的实际角度） 这个在初始化里面设置了
float  AngleIntegral=0.0; //互补滤波中积分角度

int ICM20602_Offset_Finished;

 ICM_rain ICM_Start1;
 ICM_rain ICM_Offset;
 ICM_rain ICM_Real;







/******************角度更新融合*************
名称：采集的角度的处理
说明：该函数包含了对角度和加速度进行归零处理、归一化处理、角度融合
思路： 采集值→→→→调零位→→→→归一化→→→→融合 →→→→积分 →→→→最终角度
                                          ↘    ↗
                                            补偿（通过实时的速度与积分的角度作一个跟踪互补）
*/
void Complementary_Filter(float acc_angle,float gyro_angle)
{
    float  fDeltaValue = 0;                              //补偿量
  Filte_Angle = AngleIntegral;                                 //最终融合角度
  fDeltaValue = ( acc_angle - Filte_Angle) / CONSTANT;        //时间系数矫正、补偿量
  AngleIntegral += (gyro_angle + fDeltaValue) * DT; //角速度积分融合后的角度
}


/***************************************************************
void ACC_GYRO_Offset_Get(void)
@Fucation ：零偏
@Author ：混子
@Data ：  2021-3-15
@Note ： 初始化调用
***************************************************************/

void ACC_GYRO_Offset_Get(void)
{
        uint8 i, Count = 200;

      int64 temp[6] = {0};
        ICM_Offset.gyro.x =0;
        ICM_Offset.gyro.y=0;
        ICM_Offset.gyro.z=0;

       for (i = 0; i < Count; i++)
    {
             get_icm20602_accdata_spi();//加速度计
             get_icm20602_gyro_spi();//陀螺仪
             systick_delay_ms(STM0,2);

              temp[0] += ICM_Start1.acc.x;
              temp[1] += ICM_Start1.acc.y;
              temp[2] += ICM_Start1.acc.z;

              temp[3] += ICM_Start1.gyro.x;
              temp[4] += ICM_Start1.gyro.y;
              temp[5] += ICM_Start1.gyro.z;
    }
    ICM_Offset.acc.x = temp[0] / Count;
    ICM_Offset.acc.y = temp[1] / Count;
    ICM_Offset.acc.z = temp[2] / Count;

    ICM_Offset.gyro.x = temp[3] / Count;
    ICM_Offset.gyro.y = temp[4] / Count;
    ICM_Offset.gyro.z = temp[5] / Count;
        ICM20602_Offset_Finished=1;
}

/***************************************************************
@Fucation ： 数据滑动滤波
@Author ：混子
@Data ：
@Note ：如果使用的陀螺仪差不多的话，下面的系数可以不用修改了，上述互补滤波时仅需调节Dt就可以啦！根据实际情况修改！！！
***************************************************************/
#define AcceRatio   16384.0f
#define GyroRatio   16.4f
#define ACC_FILTER_NUM 5        // 加速度计滤波深度
#define GYRO_FILTER_NUM 1       // 陀螺仪滤波深度

int32 ACC_X_BUF[ACC_FILTER_NUM], ACC_Y_BUF[ACC_FILTER_NUM], ACC_Z_BUF[ACC_FILTER_NUM];  // 滤波缓存数组
int32 GYRO_X_BUF[GYRO_FILTER_NUM], GYRO_Y_BUF[GYRO_FILTER_NUM], GYRO_Z_BUF[GYRO_FILTER_NUM];


void Data_Filter(void)  // 数据滤波
{
    int i;
    int64 temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0, temp5 = 0, temp6 = 0;

    ACC_X_BUF[0] = ICM_Start1.acc.x; // 更新滑动窗口数组
    ACC_Y_BUF[0] = ICM_Start1.acc.y;
    ACC_Z_BUF[0] = ICM_Start1.acc.z;
    GYRO_X_BUF[0] = ICM_Start1.gyro.x;
    GYRO_Y_BUF[0] = ICM_Start1.gyro.y;
    GYRO_Z_BUF[0] = ICM_Start1.gyro.z;

    for(i=0;i<ACC_FILTER_NUM;i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];

    }
    for(i=0;i<GYRO_FILTER_NUM;i++)
    {
        temp4 += GYRO_X_BUF[i];
        temp5 += GYRO_Y_BUF[i];
        temp6 += GYRO_Z_BUF[i];
    }

    ICM_Real.acc.x = temp1 / ACC_FILTER_NUM / AcceRatio;
    ICM_Real.acc.y = temp2 / ACC_FILTER_NUM / AcceRatio;
    ICM_Real.acc.z = temp3 / ACC_FILTER_NUM / AcceRatio;
    ICM_Real.gyro.x = temp4 / GYRO_FILTER_NUM / GyroRatio;
    ICM_Real.gyro.y = temp5 / GYRO_FILTER_NUM / GyroRatio;
    ICM_Real.gyro.z = temp6 / GYRO_FILTER_NUM / GyroRatio;

    for(i = 0; i < ACC_FILTER_NUM - 1; i++)
    {
        ACC_X_BUF[ACC_FILTER_NUM-1-i] = ACC_X_BUF[ACC_FILTER_NUM-2-i];
        ACC_Y_BUF[ACC_FILTER_NUM-1-i] = ACC_Y_BUF[ACC_FILTER_NUM-2-i];
        ACC_Z_BUF[ACC_FILTER_NUM-1-i] = ACC_Z_BUF[ACC_FILTER_NUM-2-i];

    }
    for(i = 0; i < GYRO_FILTER_NUM - 1; i++)
    {
        GYRO_X_BUF[GYRO_FILTER_NUM-1-i] = GYRO_X_BUF[GYRO_FILTER_NUM-2-i];
        GYRO_Y_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Y_BUF[GYRO_FILTER_NUM-2-i];
        GYRO_Z_BUF[GYRO_FILTER_NUM-1-i] = GYRO_Z_BUF[GYRO_FILTER_NUM-2-i];
    }
}
/***************************************************************
@Fucation ： 非矩阵卡尔曼滤波，这些参数不用改
@Author ：
@Data ：
@Note ：ICM_Real.gyro.y注意修改
***************************************************************/

//非矩阵卡尔曼滤波，这些参数不用改
#define Peried 1/500.0f     //卡尔曼积分周期
#define Q 2.0f              //过程噪声2.0       越小积分越慢，跟踪加速度计越慢越平滑
#define R 1000.0f           //测量噪声5000.0    越小跟踪加速度计越快
float KalmanGain = 1.0f;    //卡尔曼增益

void KalmanFilter(float ACC_Angle)
{
    //卡尔曼滤波局部参量
    static float Priori_Estimation = 0;//先验估计
    static float Posterior_Estimation = 0;//后验估计
    static float Priori_Convariance = 0;//先验方差
    static float Posterior_Convariance = 0;//后验方差

    //卡尔曼滤波
    //1.时间更新(预测) : X(k|k-1) = A(k,k-1)*X(k-1|k-1) + B(k)*u(k)
      Priori_Estimation = Posterior_Estimation - ICM_Real.gyro.y*Peried;        //先验估计，积分获得角度
    if (Priori_Estimation != Priori_Estimation)
    {
        Priori_Estimation = 0;
    }

    //2.更新先验协方差  : P(k|k-1) = A(k,k-1)*P(k-1|k-1)*A(k,k-1)'+Q(k)
    Priori_Convariance = (float)sqrt( Posterior_Convariance * Posterior_Convariance + Q * Q );
    if (Priori_Convariance != Priori_Convariance)
    {
        Priori_Convariance = 0;
    }

    //  卡尔曼后验估计：测量更新
    // 1.计算卡尔曼增益  : K(k) = P(k|k-1)*H(k)' / (H(k)*P(k|k-1)*H(k)' + R(k)) /
    KalmanGain = (float)sqrt( Priori_Convariance * Priori_Convariance / ( Priori_Convariance * Priori_Convariance + R * R ) );
    if (KalmanGain != KalmanGain)
    {
        KalmanGain = 1;
    }

    //2.测量更新(校正): X(k|k) = X(k|k-1)+K(k)*(Z(k)-H(k)*X(k|k-1))
    Posterior_Estimation  = Priori_Estimation + KalmanGain * (ACC_Angle - Priori_Estimation );
    if (Posterior_Estimation != Posterior_Estimation)
    {
        Posterior_Estimation = 0;
    }

    // 3.更新后验协方差  : P(k|k) =（I-K(k)*H(k)）*P(k|k-1)
    Posterior_Convariance = (float)sqrt(( 1 - KalmanGain ) * Priori_Convariance * Priori_Convariance );
    if (Posterior_Convariance != Posterior_Convariance)
    {
        Posterior_Convariance = 0;
    }

    //得到最终角度
    Filte_Angle = Posterior_Estimation;

    if (Filte_Angle != Filte_Angle)
    {
        Filte_Angle= 1;
    }
}

void Get_Attitude(void)
{
        angleAy = (atan2(ICM_Start1.acc.x,ICM_Start1.acc.z)) * 180 / 3.14;  //求反正切值，转化为角度
     // KalmanFilter(angleAy); //卡尔曼滤波求出滤波角度
        Complementary_Filter(angleAy,-ICM_Real.gyro.y);//互补滤波求出滤波角度
}




