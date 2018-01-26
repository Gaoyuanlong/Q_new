#include "Position.h"

void Position_Updata(u16 Time_Ms);
void Position_Init(void);
Filter_Balance FIL_Speed(0.1,0);
Filter_Balance FIL_Position_xyz(1,0);
struct Position_ Position = 
{
	Vector(0,0,0),
	Vector(0,0,0),
	Vector(0,0,0),
	Position_Init,
	Position_Updata
};

//Time :MS
void Position_Init(void)
{
	MS5611.Init();
	GPS_UART.Init(9600);
}

/****气压计三阶互补滤波方案——参考开源飞控APM****/
#define TIME_CONTANST_ZER 2.5f
#define K_ACC_ZER 	    (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))														
#define K_POS_ZER         (2.0f / TIME_CONTANST_ZER)
#define Delay_Cnt 60
void Altitude_Update(u16 Time_Ms,Vector ACC_Earth)
{
	float Origion_Acc_z;
	static float acc_correction = 0;
	static float vel_correction = 0;
	static float pos_correction = 0;
	static float Last_Acc_z = 0;
	static float Origion_Speed_z = 0;
	static float Origion_Position_z = 0;
	float SpeedDealt = 0;
	float Altitude_Dealt=0;
	float Altitude_Estimate=0;
	static float Altitude_History[Delay_Cnt+1];
	float Delta_T;
	u16 Cnt=0;
	
	static uint16_t Save_Cnt=0;
	Save_Cnt++;//数据存储周期
	
	Delta_T = Time_Ms/1000.0;//ms转s 单位 s
	Altitude_Estimate = MS5611.Data->Altitude*100.0;//高度观测量 m转cm 单位cm
	Origion_Acc_z = ACC_Earth.z*100.0;//加速度 m转cm 单位cm/s
	
	//由观测量（气压计）得到状态误差
	Altitude_Dealt = Altitude_Estimate -  Position.Position_xyz.z;//气压计(超声波)与SINS估计量的差，单位cm
	//三路积分反馈量修正惯导
	acc_correction +=Altitude_Dealt* K_ACC_ZER*Delta_T ;//加速度矫正量
	vel_correction +=Altitude_Dealt* K_VEL_ZER*Delta_T ;//速度矫正量
	pos_correction +=Altitude_Dealt* K_POS_ZER*Delta_T ;//位置矫正量
	//加速度计矫正后更新
	Last_Acc_z = Position.Acc.z;//上一次加速度量
	Position.Acc.z = Origion_Acc_z + acc_correction;// 加速度单位cm/s
	//速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
	//这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
	SpeedDealt = (Last_Acc_z + Position.Acc.z) * Delta_T/2.0;
	//原始位置更新
	Origion_Position_z += (Position.Speed.z + 0.5*SpeedDealt) * Delta_T;
	//位置矫正后更新
	Position.Position_xyz.z = Origion_Position_z + pos_correction;    
	//原始速度更新
	Origion_Speed_z += SpeedDealt;
	//速度矫正后更新
	Position.Speed.z = Origion_Speed_z + vel_correction;

	if(Save_Cnt>=1)
	{
		for(Cnt = Delay_Cnt;Cnt > 0;Cnt--)//滑动
		{
			Altitude_History[Cnt]=Altitude_History[Cnt-1];
		}
		Altitude_History[0] = Position.Position_xyz.z;
		Save_Cnt=0;
	}
/*
	static float position_old_speed = 0;
	//加速度校准
	Position.Acc.z = Origion_Acc_z*100 ;//(1 - GET_ACC_FILITER)*Position.Acc.z + GET_ACC_FILITER*Origion_Acc_z*100 ;
	User_Data.Data8 = Position.Acc.z;
	Position.Acc.z = (1 - ACC_FILITER)*Position.Acc.z+ ACC_FILITER*0;
	//速度校准
	Position.Speed.z += Position.Acc.z* Delta_T;
	
	position_old_speed = Math.Constrain(abs(position_old_speed)*100.0,1000,100);
	Position.Speed.z = ((position_old_speed- SPEED_FILITER)/position_old_speed)*Position.Speed.z + (SPEED_FILITER/position_old_speed)*0;
	//高度位置校准
	float position_old = Position.Position_xyz.z;
	Position.Position_xyz.z += Position.Speed.z* Delta_T*2;
	float temp = Math.Constrain(abs(Position.Speed.z),500,1);
	//Position.Position_xyz.z = (1-POSITION_FILITER)*Position.Position_xyz.z + (POSITION_FILITER)*Altitude_Estimate;
	Position.Position_xyz.z = ((temp - POSITION_FILITER)/temp)*Position.Position_xyz.z + (POSITION_FILITER/temp)*Altitude_Estimate;
	position_old_speed = Position.Position_xyz.z - position_old;
*/	
}

void XY_Update(u16 Time_Ms,Vector ACC_Earth)
{
	
	
	
	
}
void Position_Updata(u16 Time)
{
	Vector ACC_Earth;
	ACC_Earth = Math.Body_To_Earth(MPU6050.Data->ACC_ADC,Attitude.Angle->y,Attitude.Angle->x);
	
	ACC_Earth.x /= 4095;
	ACC_Earth.x *= 9.8f;
	
	ACC_Earth.y /= 4095;
	ACC_Earth.y *= 9.8f;
	
	ACC_Earth.z -= 4095;
	ACC_Earth.z /= 4095;
	ACC_Earth.z *= 9.8f;
//--------------高度融合--------------------------------------------------------------------//
	Altitude_Update(Time,ACC_Earth);
	
//	User_Data.Data7 = ACC_Earth.x*100;
//	User_Data.Data8 = ACC_Earth.y*100;
//	User_Data.Data9 = ACC_Earth.z*100;
}





