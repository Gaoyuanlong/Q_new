#include "FlyControl.h"

#define ATT_ANGLE_SPEED_RC_Z_MAX  20    						 //ң��Z���ת���ٶ�
#define ATT_ANGLE_SPEED      (45.0f * DEG_TO_RAD)    //���ת���ٶ�
#define ATT_ANGLE_MAX        20.0f                   //������

#define ATT_FILTER_ANGLE   	 0.1f                    //�ǶȻ�ǰ���˲���ϵ��
#define ATT_FILTER_SPEED     0.1f                    //���ٶȻ�ǰ���˲�ϵ��

#define POS_POS_SET_MAX_Z        500.0f                   //Z���߶� cm
#define POS_SPEED_SET_MAX_Z      40.0f                    //Z����ٶ� cm/s
#define POS_ACC_SET_MAX_Z        60.0f                    //Z�����ٶ� cm/s
#define POS_OUT_MAX_Z        600.0f                   //Z������


#define POS_POS_FEEBACK_MAX_Z        (1.5*POS_POS_SET_MAX_Z)    //Z������߶� cm
#define POS_SPEED_FEEBACK_MAX_Z      40.0f                    //Z����ٶ� cm/s
#define POS_ACC_FEEBACK_MAX_Z        100.0f                   //Z�����ٶ� cm/s

#define POS_FILTER_POS   	   0.1f                    //λ��ǰ���˲���ϵ��
#define POS_FILTER_SPEED     0.1f                    //�ٶȻ�ǰ���˲�ϵ��
#define POS_FILTER_ACC       0.8f                    //���ٶȻ�ǰ���˲�ϵ��

#define THROTTLE_MIN PWM_RC_MIN
#define THROTTLE_MAX PWM_RC_MAX
#define THROTTLE_MID ((PWM_RC_MAX + PWM_RC_MIN) / 2)
#define THROTTLE_60_PERCENT ((THROTTLE_MAX - THROTTLE_MIN) * 0.6f)


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
//	PID(160,20,10,100),
//	PID(160,20,10,100),
//	PID(400,0,0,0),
//	
//	PID(15,0,0,0),
//	PID(15,0,0,0),
//	PID(1,0,0,0),
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//����Ƶ��200HZ ��ֹƵ�� 2HZ 
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0,Filter_2nd(0.0009446918438402,0.00188938368768,0.0009446918438402,-1.911197067426,0.9149758348014)),	//����Ƶ��200HZ ��ֹƵ�� 2HZ 
//	
//	PID(0,0,0,0),
//	PID(0,0,0,0),
//	PID(1,0,0,0),


	PID(95,40,6,100),
	PID(90,40,5,100),
	PID(400,0,0,0),
	
	PID(10,0,0,0),
	PID(10,0,0,0),
	PID(1,0,0,0),
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(0.8,1,0,600,Filter_2nd(0.00015514842347569903,0.00031029684695139806,0.00015514842347569903,-1.964460580205232,0.96508117389913495)),	//����Ƶ��500HZ ��ֹƵ�� 2HZ 
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(1,0,0,0,Filter_2nd(0.06745527388907,0.1349105477781,0.06745527388907,-1.14298050254,0.4128015980962)),	//����Ƶ��200HZ ��ֹƵ�� 20HZ 
	
	PID(0,0,0,0),
	PID(0,0,0,0),
	PID(1,0,0,0),

	THROTTLE_MIN,
	
	0,

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
	�����Ƶ�λ 
*/
Filter_2nd Rate_Filter(0.0036216815149286421f,0.0072433630298572842f,0.0036216815149286421f,-1.8226949251963083f,0.83718165125602262f);
void ATT_Inner_Loop(u32 Time)
{
	Vector Rate_filter_Temp;
	Vector Inner_Output;
	if(Control_Para.IsLock == True)            //ͣ�� ��λ
	{
		Control_Para.ATT_Inner_PID_x.Rst_I();
		Control_Para.ATT_Inner_PID_y.Rst_I();
		Control_Para.ATT_Inner_PID_z.Rst_I();
		Motor.Stop();
		return;
	}

	//Rate_filter_Temp = Rate_Filter.LPF2ndFilter(*Attitude.Rate);
	//�����Ƕ�ȡ��������ֱ�������ڻ�����
	Control_Para.ATT_Inner_PID_x.Feedback = Attitude.Rate->x;
	Control_Para.ATT_Inner_PID_y.Feedback = Attitude.Rate->y;
	Control_Para.ATT_Inner_PID_z.Feedback = Attitude.Rate->z;
		
	Inner_Output.x = Control_Para.ATT_Inner_PID_x.Cal_PID_POS(Time);
	Inner_Output.y = Control_Para.ATT_Inner_PID_y.Cal_PID_POS(Time);
 	Inner_Output.z = Control_Para.ATT_Inner_PID_z.Cal_PID_POS(Time);
		
	//�������
	Motor.PWM->PWM1 = - Inner_Output.x -  Inner_Output.y + Inner_Output.z + Control_Para.Throttle;	
	Motor.PWM->PWM2 = - Inner_Output.x +  Inner_Output.y - Inner_Output.z + Control_Para.Throttle;
	Motor.PWM->PWM3 = + Inner_Output.x +  Inner_Output.y + Inner_Output.z + Control_Para.Throttle; 
	Motor.PWM->PWM4 = + Inner_Output.x -  Inner_Output.y - Inner_Output.z + Control_Para.Throttle;
	
	Motor.Output(True);
}
/*
	�����Ƶ�λ 
*/
void ATT_Outer_Loop(u32 Time)
{
	double ATT_Outer_PID_z_Feedback_2_Setpoint = 0;
	Vector XY_RC;
	Vector Outer_Output;
	float Z_Angle_Change;
	
	if(Control_Para.IsLock == True) 
	{
		Control_Para.ATT_Outer_PID_x.Rst_I();
		Control_Para.ATT_Outer_PID_y.Rst_I();
		Control_Para.ATT_Outer_PID_z.Rst_I();
		Control_Para.Home_Z_Angle = Attitude.Angle->z;
		Control_Para.ATT_Outer_PID_z.Setpoint = Radians(Attitude.Angle->z);	 //����ʱƫ���Ǳ���
		return;
	}
	//-----------XY��̬����-----------------------------------------------------//	
	//Xǰ����ת Y������ת ��ͷģʽң����ת��	
	Z_Angle_Change = Attitude.Angle->z - Control_Para.Home_Z_Angle; //180�Ƕ��������
	if(Z_Angle_Change > 180 )
		Z_Angle_Change = Z_Angle_Change - 360;
	else if(Z_Angle_Change < -180 )
		Z_Angle_Change = Z_Angle_Change + 360;
	else if(abs(Z_Angle_Change) < 180 )
		Z_Angle_Change = Z_Angle_Change;
	
	XY_RC.x = Radians((float)Math.Dead_Zone(PWM_RC_F_B - PWM_RC_MID,10) / PWM_RC_RANGE * ATT_ANGLE_MAX);
	XY_RC.y = Radians((float)Math.Dead_Zone(PWM_RC_L_R - PWM_RC_MID,10) / PWM_RC_RANGE * ATT_ANGLE_MAX);
	XY_RC = Math.XY_Coordinate_Rotate(XY_RC.x,XY_RC.y,Z_Angle_Change);	
	
	Control_Para.ATT_Outer_PID_x.Setpoint = (1 - ATT_FILTER_ANGLE) * Control_Para.ATT_Outer_PID_x.Setpoint + ATT_FILTER_ANGLE * XY_RC.x;
  Control_Para.ATT_Outer_PID_y.Setpoint = (1 - ATT_FILTER_ANGLE) * Control_Para.ATT_Outer_PID_y.Setpoint + ATT_FILTER_ANGLE * XY_RC.y;

	Control_Para.ATT_Outer_PID_x.Feedback = Radians(Attitude.Angle->x);
	Control_Para.ATT_Outer_PID_y.Feedback = Radians(Attitude.Angle->y);
	
	// PID���㼰�޷�
	Outer_Output.x = Math.Constrain(Control_Para.ATT_Outer_PID_x.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);
	Outer_Output.y = Math.Constrain(Control_Para.ATT_Outer_PID_y.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);
	
	// ���ƽ��
	Control_Para.ATT_Inner_PID_x.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_x.Setpoint + ATT_FILTER_SPEED * Outer_Output.x;
	Control_Para.ATT_Inner_PID_y.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_y.Setpoint + ATT_FILTER_SPEED * Outer_Output.y;	
	
	//------------Z��̬����-----------------------------------------------------//	
	Control_Para.ATT_Outer_PID_z.Feedback = Radians(Attitude.Angle->z);		
	ATT_Outer_PID_z_Feedback_2_Setpoint = Control_Para.ATT_Outer_PID_z.Feedback; //+-180�ȴ���ǰȡֵ����֤Ŀ��ֵΪ+-180��Χ
	
	//ƫ���ǿ�Խ+-180�ȴ���
	if((Control_Para.ATT_Outer_PID_z.Setpoint - Control_Para.ATT_Outer_PID_z.Feedback) > M_PI)
	{
		Control_Para.ATT_Outer_PID_z.Feedback = Control_Para.ATT_Outer_PID_z.Feedback + 2*M_PI;
	}
	else if((Control_Para.ATT_Outer_PID_z.Setpoint - Control_Para.ATT_Outer_PID_z.Feedback) < -M_PI)
	{
		Control_Para.ATT_Outer_PID_z.Feedback = Control_Para.ATT_Outer_PID_z.Feedback - 2*M_PI;
	}
	
	Outer_Output.z = Math.Constrain(Control_Para.ATT_Outer_PID_z.Cal_PID_POS(Time),ATT_ANGLE_SPEED,-ATT_ANGLE_SPEED);

	//���ݴ��������ƫ����
	PWM_In_POS Yaw_Control_Choose = PWM_In.POS_Judge(PWM_RC_Lr_Rr);	
	if(Yaw_Control_Choose != PWM_In_Mid )//����򲻽��нǶȿ��� ��ֱ���ô�������ƽ��ٶ�   ��������˳������ֻ���ʱ�Ƕ� 
	{
		Control_Para.ATT_Outer_PID_z.Setpoint = ATT_Outer_PID_z_Feedback_2_Setpoint;
		Control_Para.ATT_Inner_PID_z.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_z.Setpoint + ATT_FILTER_SPEED * Radians((float)Math.Dead_Zone(PWM_RC_MID - PWM_RC_Lr_Rr ,10) / PWM_RC_RANGE * ATT_ANGLE_SPEED_RC_Z_MAX);
	}
	else//�������ֱ���� �⻷�Ƕȿ�������� ���� ���ٶ�
	{
		Control_Para.ATT_Inner_PID_z.Setpoint = (1 - ATT_FILTER_SPEED) * Control_Para.ATT_Inner_PID_z.Setpoint + ATT_FILTER_SPEED * Outer_Output.z;
	}
}

void POS_Acc_Loop(u32 Time)
{
	//-------------Ԥ�ȴ���-------------------------------------------------------------------//
	if(Control_Para.IsLock == True)  
	{
		Control_Para.POS_Acc_PID_z.Rst_I();
		return;
	}
	//��̬ģʽ����ֱ��ȡ����ң�������źţ�����ģʽ����ȡ����PID������	
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.Throttle = (1 - POS_FILTER_ACC)*Control_Para.Throttle + POS_FILTER_ACC*2*(PWM_RC_D_U - PWM_RC_MID);		
		Control_Para.Throttle = Math.Constrain(Control_Para.Throttle,THROTTLE_60_PERCENT,0);
		return;		
	}
	//--------------pid����\ֱ�����------------------------------------------------------------------//
	Control_Para.POS_Acc_PID_z.Feedback = Math.Constrain( Position.Acc.z,POS_ACC_FEEBACK_MAX_Z,-POS_ACC_FEEBACK_MAX_Z);
	// PID���㼰�޷�
	Control_Para.Throttle = Math.Constrain(Control_Para.POS_Acc_PID_z.Cal_PID_POS_BT_LPF(Time),THROTTLE_60_PERCENT,0);
	// ���ƽ��
}

void POS_Inner_Loop(u32 Time)
{
	Vector Inner_Output;
	//-------------Ԥ�ȴ���-------------------------------------------------------------------//
	if(Control_Para.IsLock == True)  
	{
		Control_Para.POS_Inner_PID_z.Rst_I();
		return;
	}
	//��̬ģʽ����ֱ��ȡ����ң�������źţ�����ģʽ����ȡ����PID������	
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.POS_Inner_PID_z.Rst_I();		
		return;		
	}
	//--------------pid����------------------------------------------------------------------//
	Control_Para.POS_Inner_PID_z.Feedback = Math.Constrain(Position.Speed.z,POS_SPEED_FEEBACK_MAX_Z,-POS_SPEED_FEEBACK_MAX_Z);
	// PID���㼰�޷�
	Inner_Output.z =  Math.Constrain(Control_Para.POS_Inner_PID_z.Cal_PID_POS(Time),POS_ACC_SET_MAX_Z,-POS_ACC_SET_MAX_Z);	
	//--------------�������------------------------------------------------------------------//
	// ���ƽ��
	Control_Para.POS_Acc_PID_z.Setpoint = (1 - POS_FILTER_ACC) * Control_Para.POS_Acc_PID_z.Setpoint + POS_FILTER_ACC * Inner_Output.z;


//	//��̬ģʽ����ֱ��ȡ����ң�������źţ�����ģʽ����ȡ����PID������	
//	if(Control_Para.Mode == ATT) 
//	{
//		Control_Para.Throttle = (1 - POS_FILTER_ACC)*Control_Para.Throttle + POS_FILTER_ACC*PWM_RC_D_U;		
//		return;		
//	}
//	else
//	{
//		Control_Para.POS_Inner_PID_z.Feedback = Position.Speed.z;
//		// PID���㼰�޷�
//		Control_Para.Throttle = Math.Constrain(Control_Para.POS_Inner_PID_z.Cal_PID_POS(Time),THROTTLE_50_PERCENT,0) + Math.Constrain(PWM_RC_D_U,THROTTLE_MID,0);
//		// ���ƽ��
//	}		
}



// �����ķ�ʽ���ڼ򵥣��ڷ��й�����Ҳ���������������͵���������ܼ򵥵ĸ�������ֵ����
void POS_Outer_Loop(u32 Time)
{
	Vector Outer_Output;
	//-------------Ԥ�ȴ���-------------------------------------------------------------------//
	if(Control_Para.IsLock == True) 
	{
		Control_Para.POS_Outer_PID_z.Rst_I();

		Control_Para.Home = Position.Position_xyz;
		Control_Para.POS_Outer_PID_z.Setpoint = Control_Para.Home.z;		
		return;
	}
	//ʵʱ��¼��̬ģʽ�£��������߶ȣ���Ӧ���ڷ���������̬ģʽ�л�������ģʽ
	if(Control_Para.Mode == ATT) 
	{
		Control_Para.POS_Outer_PID_z.Setpoint = Position.Position_xyz.z;
		Control_Para.POS_Outer_PID_z.Rst_I();
		return;
	}
	//--------------pid����------------------------------------------------------------------//
	// Z PID����
	Control_Para.POS_Outer_PID_z.Setpoint = Math.Constrain(Control_Para.POS_Outer_PID_z.Setpoint,	Control_Para.Home.z + POS_POS_SET_MAX_Z,Control_Para.Home.z);// ���и߶��޷�  ��̬ģʽ���ɷɸߣ���̬ģʽ�л�������ģʽ��������Σ�գ�
	Control_Para.POS_Outer_PID_z.Feedback = Math.Constrain(Position.Position_xyz.z,Control_Para.Home.z + POS_POS_FEEBACK_MAX_Z,Control_Para.Home.z);
	Outer_Output.z = Math.Constrain(Control_Para.POS_Outer_PID_z.Cal_PID_POS(Time),POS_SPEED_SET_MAX_Z,-POS_SPEED_SET_MAX_Z);
	//--------------�������------------------------------------------------------------------//
	// Z ���ѡ�� ��ͣ��ң��
	PWM_In_POS Position_Control_Choose = PWM_In.POS_Judge(PWM_RC_D_U);
	if(Position_Control_Choose != PWM_In_Mid)
	{
		Control_Para.POS_Outer_PID_z.Setpoint = Position.Position_xyz.z;
		Control_Para.POS_Inner_PID_z.Setpoint = (1 - POS_FILTER_SPEED)*Control_Para.POS_Inner_PID_z.Setpoint + POS_FILTER_SPEED * ((PWM_RC_D_U - PWM_RC_MID)/PWM_RC_RANGE*2*POS_SPEED_SET_MAX_Z);
	}
	else
	{
		Control_Para.POS_Inner_PID_z.Setpoint = (1 - POS_FILTER_SPEED)*Control_Para.POS_Inner_PID_z.Setpoint + POS_FILTER_SPEED * Outer_Output.z;
	}
}


