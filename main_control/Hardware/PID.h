#ifndef __PID_H
#define __PID_H

typedef struct
{
	float SetSpeed;        
	float ActualSpeed;
	float err;
	float err_next;
	float err_last;
	float OutPut;
	float OutPut1;
	float Kp,Ki;

  int IncrementRate;

}PID_DUNC;

void PID_Init(PID_DUNC *pid);
void PID_realize(PID_DUNC *pid,float Current);

#endif

