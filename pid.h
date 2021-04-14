#ifndef __QQ_PID_H__
#define __QQ_PID_H__
/* Includes-----------------*/
#include <math.h>
/*参数初始化-------------------------------*/  
enum{     
 LLAST	= 0,     
 LAST 	= 1,    
  NOW 	= 2,          
  Position_Pid,    
  Delta_Pid,  
  }; 
 typedef struct __pid_t 
 {      
float p;     
 float i;    
  float d;          
  float set[3];			
	//目标值,包含NOW， LAST， LLAST上上次  
    float get[3];		
		//测量值     
 float err[3];			
	//误差             
   float pout;			
	//p输出    
  float iout;			
	//i输出     
 float dout;			
	//d输出        
  float out;			   
     //pid输出               
   float pos_out;		
		//本次位置式输出   
   float last_pos_out;			
//上次输出    
  float delta_out;		
		//本次增量值      
float last_delta_out;		
	//本次增量式输出 = last_delta_out + delta_u       
  float max_err;           
  unsigned int pid_mode;      
 unsigned int MaxOutput;	
			//输出限幅     
  unsigned int IntegralLimit;		
//积分限幅              
  void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化         
           unsigned int pid_mode,       
             unsigned int maxOutput,    
                unsigned int integralLimit,  
                  float p,          
          float i,                  
  float d);   
 void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);	
	//pid三个参数修改  }pid_t;     
     typedef struct{   
 float classis_get[4];   
 float classis_out[4];  
  float target[4];}chassis_t;//函数声明部分
void PID_struct_init(    pid_t* pid,    unsigned int mode,    unsigned int maxout,    unsigned int intergral_limit,        float 	kp,     float 	ki,     float 	kd);
static void pid_reset(pid_t *pid, float kp, float ki, float kd);
static void pid_init(    pid_t *pid,      unsigned int  mode,     unsigned int  maxout,     unsigned int intergral_limit,    float 	kp,     float 	ki,     float 	kd);
extern pid_t pid_spd[4] ;//四个电机的PID数据存储数组
extern chassis_t chassis;


#endif
