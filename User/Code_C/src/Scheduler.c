#include "Scheduler.h"
void Loop_1000Hz(u16 Time);
void Loop_500Hz(u16 Time);
void Loop_200Hz(u16 Time);
void Loop_50Hz(u16 Time);

struct Scheduler_ Scheduler = 
{
	Loop_1000Hz,
	Loop_500Hz,
	Loop_200Hz,
	Loop_50Hz
};

void Loop_1000Hz(u16 Time_Ms)
{
	
}

void Loop_500Hz(u16 Time_Ms)
{
  MPU6050.Updata();									//mpu6050���ݲɼ�
	Attitude.Updata();								//��̬����
	FlyControl.ATT_InnerLoop(Time_Ms);//��̬�����ڻ�
	FlyControl.POS_AccLoop(Time_Ms);		//λ�ÿ��Ƽ��ٶȻ�
}

void Loop_200Hz(u16 Time_Ms)
{
	Position.Updata(Time_Ms);							//λ�ý���
	HMC5883.Updata();											//���������ݲɼ�
	FlyControl.ATT_OuterLoop(Time_Ms);		//��̬�����⻷
  FlyControl.POS_InnerLoop(Time_Ms);		//λ�ÿ����ٶ��ڻ�
	
}

void Loop_50Hz(u16 Time_Ms)
{
	MS5611.Updata();											//��ѹ�����ݲɼ�									
	GPS_Location.GPS_Update();
	
	PWM_In.Updata();											//ң�����ݲɼ� 
  FlyControl.POS_OuterLoop(Time_Ms);		//λ�ÿ����⻷
	
	Power.Updata();										//��ѹ���ݲɼ�
	SystemState.Updata(Time_Ms);			//ϵͳ״̬���	
	Communication.UpData();						//����ͨ��	
	
//	Motor.PWM->PWM1 = 2*PWM_RC_D_U;
//	Motor.PWM->PWM2 = 2*PWM_RC_D_U;	
//	Motor.PWM->PWM3 = 2*PWM_RC_D_U;
//	Motor.PWM->PWM4 = 2*PWM_RC_D_U;
//	Motor.Output(False);
}



