#include "math.h"

typedef struct
{
	//相关速度PID参数
	float Velcity_Kp;
	float Velcity_Ki;
	float Velcity_Kd;
	float Ur;				//限幅值
	
    int EN;                 //PID使能
	int Un;					//期望输出值
	int En_1;				//上一次的误差值
	int En_2;				//上上次的误差值
	int PWM;				//输出PWM值
}PID_InitDefStruct;

void PID_Init(PID_InitDefStruct* p) //PID值初始化
{
	p->Velcity_Kp = 5.0;
	p->Velcity_Ki = 0.5;
	p->Velcity_Kd = 0.0;
	p->Ur = 1500;
	p->EN = 1;
	p->Un = 0;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
}

void Velocity_PID(int targetVelocity,int currentVelocity,PID_InitDefStruct* p)
{
	if(p->EN == 1)
	{
		int En = targetVelocity - currentVelocity;//误差值                                                     
	
		p->Un += p->Velcity_Kp*(En - p->En_1) + p->Velcity_Ki*En + p->Velcity_Kd*(En - 2*p->En_1 + p->En_2); //增量式PID
		
		p->En_2 = p->En_1;
		p->En_1 = En;
		
		p->PWM = p->Un;
		
		/*输出限幅*/
		if(p->PWM > p->Ur) p->PWM = p->Ur;
		if(p->PWM < -p->Ur) p->PWM = -p->Ur;
	}
	else
	{
		PID_Init(p);
	}
}
