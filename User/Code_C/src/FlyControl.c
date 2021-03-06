#include "FlyControl.h"

#define ATT_ANGLE_SPEED_RC_Z_MAX  20    						 //遥控Z最大转角速度
#define ATT_ANGLE_SPEED      (45.0f * DEG_TO_RAD)    //最大转角速度
#define ATT_ANGLE_MAX        20.0f                   //最大倾角

#define ATT_FILTER_ANGLE   	 0.1f                    //角度环前置滤波器系数
#define ATT_FILTER_SPEED     0.1f                    //角速度环前置滤波系数

#define POS_POS_MAX_Z        250.0f                   	//Z最大高度 cm
#define POS_SPEED_MAX_Z      20.0f                    //Z最大速度 cm/s
#define POS_ACC_MAX_Z        60.0f                    //Z最大加速度 cm/s

#define POS_FILTER_POS   	   0.1f                    //位置前置滤波器系数
#define POS_FILTER_SPEED     0.1f                    //速度环前置滤波系数
#define POS_FILTER_ACC       0.8f                    //加速度环前置滤波系数

void ATT_Inner_Loop(u32 Time);
void ATT_Outer_Loop(u32 Time);

void POS_Acc_Loop(u32 Time);
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
	PID(160,20,10,100),
	PID(160,20,10,100),
	PID(600,0,0,0),
	
	PID(15,0,0,0),
	PID(15,0,0,0),
	PID(5,0,0,0),
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//采样频率200HZ 截止频率 2HZ 
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//采样频率200HZ 截止频率 2HZ 
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(1,0,0,0),


//	PID(95,40,6,100),
//	PID(90,40,5,100),
//	PID(600,0,0,0),
//	
//	PID(10,0,0,0),
//	PID(10,0,0,0),
//	PID(5,0,0,0),
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//采样频率200HZ 截止频率 2HZ 
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//采样频率200HZ 截止频率 2HZ 
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0),

	THROTTLE_MIN,

	Vector(0,0,0),
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
	
	POS_Acc_Loop,
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
	Control_Para.Throttle += 700;
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
	double ATT_Outer_PID_z_Feedback_2_Setpoint = 0;
	Vector XY_RC;
	Vector Outer_Output;
	if(Control_Para.IsLock == True) 
	{
		Control_Para.ATT_Outer_PID_x.Rst_I();
		Control_Para.ATT_Outer_PID_y.Rst_I();
		Control_Para.ATT_Outer_PID_z.Rst_I();
		
		Control_Para.ATT_Outer_PID_z.Setpoint = Radians(Attitude.Angle->z);	 //启动时偏航角保持
		return;
	}

	//-----------XY姿态控制-----------------------------------------------------//	
	//X前后旋转 Y左右旋转 无头模式遥控量转换
	XY_RC.x = Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_F_B ,10) / PWM_RC_RANGE * ATT_ANGLE_MAX);
	XY_RC.y = Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_L_R ,10) / PWM_RC_RANGE * ATT_ANGLE_MAX);
	XY_RC = Math.XY_Coordinate_Rotate(XY_RC.x,XY_RC.y,Attitude.Angle->z);	
	
	Control_Para.ATT_Outer_PID_x.Setpoint = (1 - ATT_FILTER_ANGLE) * Control_Para.ATT_Outer_PID_x.Setpoint + ATT_FILTER_ANGLE * XY_RC.x;
  Control_Para.ATT_Outer_PID_y.Setpoint = (1 - ATT_FILTER_ANGLE) * Control_Para.ATT_Outer_PID_y.Setpoint + ATT_FILTER_ANGLE * XY_RC.y;

	Control_Para.ATT_Outer_PID_x.Feedback = Radians(Attitude.Angle->x);
	Control_Para.ATT_Outer_PID_y.Feedback = Radians(Attitude.Angle->y);
	
	// PID计算及限幅
	Outer_Output.x = Math.Constrain(Control_Para.ATT_Outer_PID_x.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);
	Outer_Output.y = Math.Constrain(Control_Para.ATT_Outer_PID_y.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);
	
	// 输出平滑
	Control_Para.ATT_Inner_PID_x.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_x.Setpoint + ATT_FILTER_SPEED * Outer_Output.x;
	Control_Para.ATT_Inner_PID_y.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_y.Setpoint + ATT_FILTER_SPEED * Outer_Output.y;	
	
	//------------Z姿态控制-----------------------------------------------------//	
	Control_Para.ATT_Outer_PID_z.Feedback = Radians(Attitude.Angle->z);		
	ATT_Outer_PID_z_Feedback_2_Setpoint = Control_Para.ATT_Outer_PID_z.Feedback; //+-180度处理前取值，保证目标值为+-180范围
	
	//偏航角跨越+-180度处理
	if((Control_Para.ATT_Outer_PID_z.Setpoint - Control_Para.ATT_Outer_PID_z.Feedback) > M_PI)
	{
		Control_Para.ATT_Outer_PID_z.Feedback = Control_Para.ATT_Outer_PID_z.Feedback + 2*M_PI;
	}
	else if((Control_Para.ATT_Outer_PID_z.Setpoint - Control_Para.ATT_Outer_PID_z.Feedback) < -M_PI)
	{
		Control_Para.ATT_Outer_PID_z.Feedback = Control_Para.ATT_Outer_PID_z.Feedback - 2*M_PI;
	}
	
	Outer_Output.z = Math.Constrain(Control_Para.ATT_Outer_PID_z.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);

	//根据打舵量控制偏航角
	PWM_In_POS Yaw_Control_Choose = PWM_In.POS_Judge(PWM_RC_Lr_Rr);	
	if(Yaw_Control_Choose != PWM_In_Mid )//打舵则不进行角度控制 并直接用打舵量控制角速度   不打舵则退出并保持回中时角度 
	{	
		Control_Para.ATT_Outer_PID_z.Setpoint = ATT_Outer_PID_z_Feedback_2_Setpoint;
		Control_Para.ATT_Inner_PID_z.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Outer_PID_z.Setpoint + ATT_FILTER_SPEED * Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_Lr_Rr ,10) / PWM_RC_RANGE * ATT_ANGLE_SPEED_RC_Z_MAX);
	}
	else//不打舵则直接用 外环角度控制输出量 控制 角速度
	{
		Control_Para.ATT_Inner_PID_z.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_z.Setpoint + ATT_FILTER_SPEED * Outer_Output.z;
	}
}

void POS_Acc_Loop(u32 Time)
{
	//-------------预先处理-------------------------------------------------------------------//
	if(Control_Para.IsLock == True)  
	{
		Control_Para.POS_Acc_PID_z.Rst_I();
		return;
	}
	//姿态模式油门直接取决于遥控输入信号，其他模式油门取决于PID运算结果	
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.Throttle = (1 - POS_FILTER_ACC)*Control_Para.Throttle + POS_FILTER_ACC*PWM_RC_D_U;		
		return;		
	}
	//--------------pid控制\直接输出------------------------------------------------------------------//
	Control_Para.POS_Acc_PID_z.Feedback = Position.Acc.z;
	// PID计算及限幅
	Control_Para.Throttle = Math.Constrain(Control_Para.POS_Acc_PID_z.Cal_PID_POS_BT_LPF(Time),THROTTLE_40_PERCENT,0) + PWM_RC_D_U;
	// 输出平滑
}

void POS_Inner_Loop(u32 Time)
{
	Vector Inner_Output;
	//-------------预先处理-------------------------------------------------------------------//
	if(Control_Para.IsLock == True)  
	{
		Control_Para.POS_Inner_PID_z.Rst_I();
		return;
	}
	//姿态模式油门直接取决于遥控输入信号，其他模式油门取决于PID运算结果	
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.POS_Inner_PID_z.Rst_I();		
		return;		
	}
	//--------------pid控制------------------------------------------------------------------//
	Control_Para.POS_Inner_PID_z.Feedback = Position.Speed.z;
	// PID计算及限幅
	Inner_Output.z =  Math.Constrain(Control_Para.POS_Acc_PID_z.Cal_PID_POS(Time),POS_ACC_MAX_Z,-POS_ACC_MAX_Z);	
	//--------------输出处理------------------------------------------------------------------//
	// 输出平滑
	Control_Para.POS_Acc_PID_z.Setpoint = (1 - POS_FILTER_ACC) * Control_Para.POS_Acc_PID_z.Setpoint + POS_FILTER_ACC * Inner_Output.z;


//	//姿态模式油门直接取决于遥控输入信号，其他模式油门取决于PID运算结果	
//	if(Control_Para.Mode == ATT) 
//	{
//		Control_Para.Throttle = (1 - POS_FILTER_ACC)*Control_Para.Throttle + POS_FILTER_ACC*PWM_RC_D_U;		
//		return;		
//	}
//	else
//	{
//		Control_Para.POS_Inner_PID_z.Feedback = Position.Speed.z;
//		// PID计算及限幅
//		Control_Para.Throttle = Math.Constrain(Control_Para.POS_Inner_PID_z.Cal_PID_POS(Time),THROTTLE_50_PERCENT,0) + Math.Constrain(PWM_RC_D_U,THROTTLE_MID,0);
//		// 输出平滑
//	}		
}

// 加锁的方式过于简单，在飞行过程中也会出现油门拉到最低的情况，不能简单的根据油门值加锁
void POS_Outer_Loop(u32 Time)
{
	Vector Outer_Output;
	//-------------预先处理-------------------------------------------------------------------//
	if(Control_Para.IsLock == True) 
	{
		Control_Para.POS_Outer_PID_z.Rst_I();

		Control_Para.Home = Position.Position_xyz;
		Control_Para.POS_Outer_PID_z.Setpoint = Control_Para.Home.z;		
		return;
	}
	//实时记录姿态模式下，飞行器高度，以应对在飞行中由姿态模式切换到定高模式
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.POS_Outer_PID_z.Setpoint = Position.Position_xyz.z;
		Control_Para.POS_Outer_PID_z.Rst_I();
		return;
	}
	//--------------pid控制------------------------------------------------------------------//
	// Z PID计算
	Control_Para.POS_Outer_PID_z.Setpoint = Math.Constrain(Control_Para.POS_Outer_PID_z.Setpoint,	Control_Para.Home.z + POS_POS_MAX_Z,Control_Para.Home.z);// 飞行高度限幅  姿态模式不可飞高！姿态模式切换到定高模式会有跳变危险！
	Control_Para.POS_Outer_PID_z.Feedback = Position.Position_xyz.z;
	Outer_Output.z = Math.Constrain(Control_Para.POS_Outer_PID_z.Cal_PID_POS(Time),POS_SPEED_MAX_Z,-POS_SPEED_MAX_Z);
	//--------------输出处理------------------------------------------------------------------//
	// Z 输出选择 悬停或遥控
	PWM_In_POS Position_Control_Choose = PWM_In.POS_Judge(PWM_RC_D_U);	
	if(Position_Control_Choose != PWM_In_Mid)
	{
		Control_Para.POS_Outer_PID_z.Setpoint = Position.Position_xyz.z;
		Control_Para.POS_Inner_PID_z.Setpoint = (1 - POS_FILTER_SPEED)*Control_Para.POS_Inner_PID_z.Setpoint + POS_FILTER_SPEED *(PWM_RC_D_U - THROTTLE_MID);
	}
	else
	{
		Control_Para.POS_Inner_PID_z.Setpoint = (1 - POS_FILTER_SPEED)*Control_Para.POS_Inner_PID_z.Setpoint + POS_FILTER_SPEED * Outer_Output.z;
	}
}
