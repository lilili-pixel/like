#include "PID.h"

float incrementSpeed=0;

void PID_Init(PID_DUNC *pid)
{
	pid->ActualSpeed = 0.9;    // ʵ��ֵ������ǰ������ֵ
	pid->SetSpeed = 0.5;         // �����趨ֵ
	pid->err = 0.0;            // ����ʽpid�����Ϊs�趨ֵ-ʵ��ֵ
	pid->err_last = 0.0;       // ��һ������� ��e(k-1)
	pid->err_next = 0.0;       // ����������� ��e(k-2)
	pid->OutPut = 0;        // PWM���ڿ�����ʵ�����ֵ

	pid->Kp = 5;           // Kp
	pid->Ki = 0.4;           // Ki
	//pid->IncrementRate = 20;   //�Ŵ�ϵ�� �ѵ�ѹ������ֵ��ռ�ձȵıȽ�ֵ��ϵ������
}

void PID_realize(PID_DUNC *pid,float Current)
{
	pid->ActualSpeed = Current;

  //pid->err=pid->SetSpeed-pid->ActualSpeed;                // ���=�趨ֵ-ʵ��ֵ
	
	pid->err=pid->ActualSpeed-pid->SetSpeed;

  incrementSpeed=pid->Kp*(pid->err-pid->err_last)+pid->Ki*pid->err;
  //incrementSpeed*=pid->IncrementRate;// ����ʽpid

  pid->OutPut += incrementSpeed;

//	pid->OutPut1 = -pid->OutPut;
//	if(pid->OutPut1 > 5200) pid->OutPut1 = 5200;
//	if(pid->OutPut1 < 100) pid->OutPut1 = 100;
	
  if (pid->OutPut > 5200)  pid->OutPut = 5200;    //�޷�
	if (pid->OutPut < 20)  pid->OutPut = 20;    //�޷�
	

  pid->err_last=pid->err;
  pid->err_next=pid->err_last;
}
