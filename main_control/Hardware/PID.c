#include "PID.h"

float incrementSpeed=0;

void PID_Init(PID_DUNC *pid)
{
	pid->ActualSpeed = 0.9;    // 实际值，即当前采样的值
	pid->SetSpeed = 0.5;         // 电流设定值
	pid->err = 0.0;            // 增量式pid，误差为s设定值-实际值
	pid->err_last = 0.0;       // 上一个的误差 即e(k-1)
	pid->err_next = 0.0;       // 上两个的误差 即e(k-2)
	pid->OutPut = 0;        // PWM周期控制量实际输出值

	pid->Kp = 5;           // Kp
	pid->Ki = 0.4;           // Ki
	//pid->IncrementRate = 20;   //放大系数 把电压采样的值和占空比的比较值联系起来。
}

void PID_realize(PID_DUNC *pid,float Current)
{
	pid->ActualSpeed = Current;

  //pid->err=pid->SetSpeed-pid->ActualSpeed;                // 误差=设定值-实际值
	
	pid->err=pid->ActualSpeed-pid->SetSpeed;

  incrementSpeed=pid->Kp*(pid->err-pid->err_last)+pid->Ki*pid->err;
  //incrementSpeed*=pid->IncrementRate;// 增量式pid

  pid->OutPut += incrementSpeed;

//	pid->OutPut1 = -pid->OutPut;
//	if(pid->OutPut1 > 5200) pid->OutPut1 = 5200;
//	if(pid->OutPut1 < 100) pid->OutPut1 = 100;
	
  if (pid->OutPut > 5200)  pid->OutPut = 5200;    //限幅
	if (pid->OutPut < 20)  pid->OutPut = 20;    //限幅
	

  pid->err_last=pid->err;
  pid->err_next=pid->err_last;
}
