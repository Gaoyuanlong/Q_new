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

void Loop_1000Hz(u16 Time)
{
	
}

void Loop_500Hz(u16 Time)
{
  MPU6050.Updata();									//mpu6050���ݲɼ�
	Attitude.Updata();								//��̬����
//Altitude.Updata(Time);						//λ�ý���
	FlyControl.ATT_InnerLoop(Time);		//��̬�����ڻ�
}

void Loop_200Hz(u16 Time)
{
	HMC5883.Updata();									//���������ݲɼ�
	FlyControl.ATT_OuterLoop(Time);		//��̬�����⻷
//FlyControl.POS_InnerLoop(Time);	//λ�ÿ����ڻ�
}

void Loop_50Hz(u16 Time)
{
	SystemState.Updata(Time);					//ϵͳ״̬���
	Communication.UpData();						//����ͨ��
//FlyControl.POS_OuterLoop(Time);		//λ�ÿ����⻷
	Power.Updata();										//��ѹ���ݲɼ�
	MS5611.Updata();									//��ѹ�����ݲɼ�									
	PWM_In.Updata();									//ң�����ݲɼ�
	
}



