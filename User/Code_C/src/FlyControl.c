#include "FlyControl.h"


#define ANGLE_SPEED      (45.0f * DEG_TO_RAD)    //最大转角速度
#define ANGLE_MAX        20.0f                   //最大倾角

#define ALT_SPEED        2.0f                    //最大升降速度 M/S
#define ALT_MAX          10.0f                   //最大高度

#define FILTER_ANGLE   	 1.0f                    //角度环前置滤波器系数
#define FILTER_SPEED     1.0f                    //角速度环前置滤波系数


void ATT_Inner_Loop(u32 Time);
void ATT_Outer_Loop(u32 Time);

void POS_Inner_Loop(u32 Time);
void POS_Outer_Loop(u32 Time);

/*
	PID ATT_Inner_PID_x;
	PID ATT_Inner_PID_y;
	PID ATT_Inner_PID_z;
	
	PID ATT_Outer_PID_x;
	PID ATT_Outer_PID_y;	
	PID ATT_Outer_PID_z;	
	
	PID POS_Inner_PID_x;
	PID POS_Inner_PID_y;
	PID POS_Inner_PID_z;
	
	PID POS_Outer_PID_x;
	PID POS_Outer_PID_y;
	PID POS_Outer_PID_z;
	int Throttle;

	float ALT_Onland;
	BOOL IsLock;
	BOOL IsLost;
	BOOL IsError;
	Fly_Mode Mode;  
*/
struct Control_Para_ Control_Para = 
{
	PID(95,40,6,100),
	PID(90,40,5,100),
	PID(600,0,0,0),
	
	PID(10,0,0,0),
	PID(10,0,0,0),
	PID(5,0,0,0),
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(0,0,0,0),
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(0,0,0,0),
	
	SBUS_MIN,

	0,
	True,
	False,
	False,
	ALT
};
	
struct FlyControl_ FlyControl = 
{	
	&Control_Para,
	
	ATT_Inner_Loop,
	ATT_Outer_Loop,

	POS_Inner_Loop,
	POS_Outer_Loop,	
	
};

/*
	弧度制单位 
*/

Filter_2nd Rate_Filter(0.0036216815149286421f,0.0072433630298572842f,0.0036216815149286421f,-1.8226949251963083f,0.83718165125602262f);
void ATT_Inner_Loop(u32 Time)
{
	Vector Rate_filter_Temp;
	Vector Inner_Output;
	if(Control_Para.IsLock == True)            //停机复位
	{
		Control_Para.ATT_Inner_PID_x.Rst_I();
		Control_Para.ATT_Inner_PID_y.Rst_I();
		Control_Para.ATT_Inner_PID_z.Rst_I();
		Motor.Stop();
		return;
	}

	//Rate_filter_Temp = Rate_Filter.LPF2ndFilter(*Attitude.Rate);
	//陀螺仪读取到的数据直接用于内环控制
	Control_Para.ATT_Inner_PID_x.Feedback = Attitude.Rate->x;
	Control_Para.ATT_Inner_PID_y.Feedback = Attitude.Rate->y;
	Control_Para.ATT_Inner_PID_z.Feedback = Attitude.Rate->z;
		
	Inner_Output.x = Control_Para.ATT_Inner_PID_x.Cal_PID_POS(Time);
	Inner_Output.y = Control_Para.ATT_Inner_PID_y.Cal_PID_POS(Time);
 	Inner_Output.z = Control_Para.ATT_Inner_PID_z.Cal_PID_POS(Time);
		
	//四轴输出
	Control_Para.Throttle = PWM_RC_D_U + 900;
	Motor.PWM->PWM1 = - Inner_Output.x -  Inner_Output.y + Inner_Output.z + Control_Para.Throttle;	
	Motor.PWM->PWM2 = - Inner_Output.x +  Inner_Output.y - Inner_Output.z + Control_Para.Throttle;
	Motor.PWM->PWM3 = + Inner_Output.x +  Inner_Output.y + Inner_Output.z + Control_Para.Throttle; 
	Motor.PWM->PWM4 = + Inner_Output.x -  Inner_Output.y - Inner_Output.z + Control_Para.Throttle;
	
	Motor.Output(True);
}
/*
	弧度制单位
*/

void ATT_Outer_Loop(u32 Time)
{
	if(Control_Para.IsLock == True) 
	{
		Control_Para.ATT_Outer_PID_x.Rst_I();
		Control_Para.ATT_Outer_PID_y.Rst_I();
		Control_Para.ATT_Outer_PID_z.Rst_I();
		return;
	}
	
	Control_Para.ATT_Outer_PID_x.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_x.Setpoint + FILTER_ANGLE * Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_F_B ,10) / PWM_RC_RANGE * ANGLE_MAX);
  Control_Para.ATT_Outer_PID_y.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_y.Setpoint + FILTER_ANGLE * Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_L_R ,10) / PWM_RC_RANGE * ANGLE_MAX);
	Control_Para.ATT_Outer_PID_z.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_z.Setpoint + FILTER_ANGLE * Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_Lr_Rr ,10) / PWM_RC_RANGE * ANGLE_MAX);
	
	Control_Para.ATT_Outer_PID_x.Feedback = Radians(Attitude.Angle->x);
	Control_Para.ATT_Outer_PID_y.Feedback = Radians(Attitude.Angle->y);
	Control_Para.ATT_Outer_PID_z.Feedback = Radians(Attitude.Angle->z);
	// PID计算及限幅
	float TMP_X = Math.Constrain(Control_Para.ATT_Outer_PID_x.Cal_PID_POS(Time),ANGLE_SPEED,-ANGLE_SPEED);
	float TMP_Y = Math.Constrain(Control_Para.ATT_Outer_PID_y.Cal_PID_POS(Time),ANGLE_SPEED,-ANGLE_SPEED);
	float TMP_Z = Math.Constrain(Control_Para.ATT_Outer_PID_z.Cal_PID_POS(Time),ANGLE_SPEED,-ANGLE_SPEED);
	// 输出平滑
	Control_Para.ATT_Inner_PID_x.Setpoint = (1 - FILTER_SPEED) * Control_Para.ATT_Inner_PID_x.Setpoint + FILTER_SPEED * TMP_X;
	Control_Para.ATT_Inner_PID_y.Setpoint = (1 - FILTER_SPEED) * Control_Para.ATT_Inner_PID_y.Setpoint + FILTER_SPEED * TMP_Y;
	Control_Para.ATT_Inner_PID_z.Setpoint = (1 - FILTER_SPEED) * Control_Para.ATT_Inner_PID_z.Setpoint + FILTER_SPEED * TMP_Z;
}

void POS_Inner_Loop(u32 Time)
{
	
//	if(Control_Para.IsLock == True)  
//	{
//		Control_Para.POS_Inner_PID_z.Rst_I();
//		Control_Para.Throttle = THROTTLE_MIN;
//		return;
//	}
//	//姿态模式油门直接取决于遥控输入信号，其他模式油门取决于PID运算结果	
//	if(Control_Para.Mode == ATT) 
//	{
//		Control_Para.Throttle = RC_THROTTLE;			
//	}
//	else
//	{
//		Control_Para.POS_Inner_PID_z.Feedback = Altitude.Speed;
//		// PID计算及限幅
//		Control_Para.Throttle = Math.Constrain(Control_Para.POS_Inner_PID_z.Cal_PID_POS(Time),THROTTLE_80_PERCENT,0) + Math.Constrain(RC_THROTTLE,THROTTLE_MID,0);
//		// 输出平滑
//	}
//	
//	Vector TMP(0,0,0);
//	if(Control_Para.IsLost == False)
//	{		
//		TMP.x = Math.Dead_Zone(RC_ROLL - RC_MID,  2) / RC_RANGE * ANGLE_MAX;
//		TMP.y = Math.Dead_Zone(RC_MID  - RC_PITCH,2) / RC_RANGE * ANGLE_MAX;
//		TMP.z = Math.Dead_Zone(RC_YAW  - RC_MID,  2) / RC_RANGE * ANGLE_SPEED;
//	}

//	Control_Para.ATT_Outer_PID_x.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_x.Setpoint + FILTER_ANGLE * TMP.x;
//	Control_Para.ATT_Outer_PID_y.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_y.Setpoint + FILTER_ANGLE * TMP.y;
//	Control_Para.ATT_Outer_PID_z.Setpoint = (1 - FILTER_ANGLE) * Control_Para.ATT_Outer_PID_z.Setpoint + FILTER_ANGLE * TMP.z;		
	
}
// 加锁的方式过于简单，在飞行过程中也会出现油门拉到最低的情况，不能简单的根据油门值加锁
void POS_Outer_Loop(u32 Time)
{
//	if(Control_Para.IsLock == True) 
//	{
//		Control_Para.POS_Outer_PID_z.Rst_I();
//		Control_Para.ALT_Onland = Altitude.Altitude;
//		Control_Para.POS_Outer_PID_z.Setpoint = Altitude.Altitude;
//		return;
//	}
//	
//	//实时记录姿态模式下，飞行器高度，以应对在飞行中由姿态模式切换到定高模式
//	if(Control_Para.Mode == ATT) 
//	{
//		Control_Para.POS_Outer_PID_z.Setpoint = Altitude.Altitude;
//		return;
//	}
//	if(Control_Para.IsLost == False)
//	{	
//		Control_Para.POS_Outer_PID_z.Setpoint += Math.Dead_Zone(RC_THROTTLE - RC_MID,2) / RC_RANGE * ALT_SPEED * Time / 1000.0f;	
//		//飞行高度限幅
//		Control_Para.POS_Outer_PID_z.Setpoint = Math.Constrain(Control_Para.POS_Outer_PID_z.Setpoint,	Control_Para.ALT_Onland + ALT_MAX,Control_Para.ALT_Onland);// 飞行高度限幅
//	}
//	else
//	{
//		//自动降落
//	}	
//	
//	Control_Para.POS_Outer_PID_z.Feedback = Altitude.Altitude;
//	// PID计算及限幅
//	Control_Para.POS_Inner_PID_z.Setpoint = Math.Constrain(Control_Para.POS_Outer_PID_z.Cal_PID_POS(Time),ALT_SPEED,-ALT_SPEED);
//	// 输出平滑
}
