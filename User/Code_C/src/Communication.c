#include "Communication.h"
#include <stdio.h>

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
#define DEBUG

/***********************通信串口通道选择**********************/
#define usart_pc usart1
/**********************************************************/


BOOL Communicate(void);
#define BUF_SIZE 64
u8 Communicate_BUF[BUF_SIZE];

struct SYS_State_ SYS_State;
struct User_Data_ User_Data;

struct Communication_ Communication = 
{
	Communicate
};

void Send_Senser_PC(void)
{
	vs16 Temp1 = 0;
	vs32 Temp2 = 0;
	u8 Cnt = 1;
	
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0X02;
	Communicate_BUF[Cnt++] = 0;
	
	Temp1 = MPU6050.Data->ACC_ADC.x;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = MPU6050.Data->ACC_ADC.y;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = MPU6050.Data->ACC_ADC.z;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = MPU6050.Data->GYR_ADC.x;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 =  MPU6050.Data->GYR_ADC.y;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 =  MPU6050.Data->GYR_ADC.z;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = HMC5883.Data->MAG_ADC.x*100;;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = HMC5883.Data->MAG_ADC.y*100;;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = HMC5883.Data->MAG_ADC.z*100;;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);
	
	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++] = Sum;
	Communicate_BUF[0] = Cnt - 1;   

	usart_pc.send(Communicate_BUF,Cnt);
	
}
void Send_Senser2_PC(void)
{
	vs32 Temp2 = 0;
	u8 Cnt = 1;
	
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0X07;
	Communicate_BUF[Cnt++] = 0;
	
	Temp2 = MS5611.Data->Altitude*10000;	
	Communicate_BUF[Cnt++] = BYTE3(Temp2);	
	Communicate_BUF[Cnt++] = BYTE2(Temp2);		
	Communicate_BUF[Cnt++] = BYTE1(Temp2);	
	Communicate_BUF[Cnt++] = BYTE0(Temp2);
	
	Temp2 = MS5611.Data->Speed*10000;
	Communicate_BUF[Cnt++] = BYTE3(Temp2);	
	Communicate_BUF[Cnt++] = BYTE2(Temp2);	
	Communicate_BUF[Cnt++] = BYTE1(Temp2);	
	Communicate_BUF[Cnt++] = BYTE0(Temp2);
	
	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++] = Sum;
	Communicate_BUF[0] = Cnt - 1;   

	usart_pc.send(Communicate_BUF,Cnt);
	
}
void Send_Eular_PC(void)
{
	u16 Temp1 = 0;
	vs32 Temp2 = 0;
	u8 Cnt = 1;

	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0X01;
	Communicate_BUF[Cnt++] = 0;
	
	Temp1 = (int)(Attitude.Angle->x * 100.0f);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = (int)(Attitude.Angle->y * 100.0f);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = (int)(Attitude.Angle->z * 100.0f);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
  Temp2 = (u32)(Position.Position_xyz.z* 100.0f);	
	Communicate_BUF[Cnt++] = BYTE3(Temp2);	
	Communicate_BUF[Cnt++] = BYTE2(Temp2);	
	Communicate_BUF[Cnt++] = BYTE1(Temp2);	
	Communicate_BUF[Cnt++] = BYTE0(Temp2);
	
	Communicate_BUF[Cnt++] = MPU6050.IsCalibrating;
	Communicate_BUF[Cnt++] = False;	

	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++] = Sum;
	Communicate_BUF[0] = Cnt - 1; 
	
	usart_pc.send(Communicate_BUF,Cnt);
}

void Send_RC_PC(void)
{
	u16 Temp1 = 0;
	u8 Cnt = 1;
	
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0X03;
	Communicate_BUF[Cnt++] = 0;
	
	Temp1 = 15 ;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = 0;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = 0;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = 0;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = PWM_In.Data->CH1;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = PWM_In.Data->CH2;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = PWM_In.Data->CH3; 
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	

	Temp1 = PWM_In.Data->CH4;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);
	
	Temp1 = PWM_In.Data->CH5;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);

	Temp1 = PWM_In.Data->CH6;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);
	Communicate_BUF[Cnt++] = BYTE0(Temp1);

	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++] = Sum;
	Communicate_BUF[0] = Cnt - 1; 
	
	usart_pc.send(Communicate_BUF,Cnt);
}

void Send_Motor_PC(void)
{
	u16 Temp1 = 0;
	u8 Cnt = 1;
	
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0XAA;
	Communicate_BUF[Cnt++] = 0X06;
	Communicate_BUF[Cnt++] = 0;
	
	Temp1 = Motor.PWM->PWM1;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = Motor.PWM->PWM2;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = Motor.PWM->PWM3;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		

	Temp1 = Motor.PWM->PWM4;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = Power.Data->BAT_3S*100;		
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i = 1;i < Cnt; i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++] = Sum;
	Communicate_BUF[0] = Cnt - 1; 
	usart_pc.send(Communicate_BUF,Cnt);
}

BOOL Send_PID_Para_PC(u8 Group)
{
	u8 Cnt = 1;
	vs16 Temp = 0;
	PID Temp_PID[3];
	
	switch(Group)
	{
		case 1:
			Temp_PID[0].Kp = FlyControl.Para->ATT_Inner_PID_x.Kp;
			Temp_PID[0].Ki = FlyControl.Para->ATT_Inner_PID_x.Ki;
			Temp_PID[0].Kd = FlyControl.Para->ATT_Inner_PID_x.Kd;
		
			Temp_PID[1].Kp = FlyControl.Para->ATT_Inner_PID_y.Kp;
			Temp_PID[1].Ki = FlyControl.Para->ATT_Inner_PID_y.Ki;
			Temp_PID[1].Kd = FlyControl.Para->ATT_Inner_PID_y.Kd;
		
			Temp_PID[2].Kp = FlyControl.Para->ATT_Inner_PID_z.Kp;
			Temp_PID[2].Ki = FlyControl.Para->ATT_Inner_PID_z.Ki;
			Temp_PID[2].Kd = FlyControl.Para->ATT_Inner_PID_z.Kd;
		break;	
		case 2:
			Temp_PID[0].Kp = FlyControl.Para->ATT_Outer_PID_x.Kp*100.0;
			Temp_PID[0].Ki = FlyControl.Para->ATT_Outer_PID_x.Ki*100.0;
			Temp_PID[0].Kd = FlyControl.Para->ATT_Outer_PID_x.Kd*100.0;
		
			Temp_PID[1].Kp = FlyControl.Para->ATT_Outer_PID_y.Kp*100.0;
			Temp_PID[1].Ki = FlyControl.Para->ATT_Outer_PID_y.Ki*100.0;
			Temp_PID[1].Kd = FlyControl.Para->ATT_Outer_PID_y.Kd*100.0;
		
			Temp_PID[2].Kp = FlyControl.Para->ATT_Outer_PID_z.Kp*100.0;
			Temp_PID[2].Ki = FlyControl.Para->ATT_Outer_PID_z.Ki*100.0;
			Temp_PID[2].Kd = FlyControl.Para->ATT_Outer_PID_z.Kd*100.0;
			break;
		case 3:
			break;
		default:
			break;
	}
	
	Communicate_BUF[Cnt++] = 0xAA;
	Communicate_BUF[Cnt++] = 0xAA;
	Communicate_BUF[Cnt++] = 0x10 + Group - 1;
	Communicate_BUF[Cnt++] = 0;
	
	Temp = Temp_PID[0].Kp ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[0].Ki ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[0].Kd ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = Temp_PID[1].Kp;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[1].Ki;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[1].Kd ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = Temp_PID[2].Kp;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[2].Ki ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	Temp = Temp_PID[2].Kd ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i=1;i<Cnt;i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++]=Sum;

	Communicate_BUF[0] = Cnt - 1;    //HID第一个字节 表示数据长度
	
	usart_pc.send(Communicate_BUF,Cnt);
	return True;
}

BOOL Send_UserData_PC(void)
{
	u8 Cnt = 1;
	vs16 Temp = 0;
	
	User_Data.Data1 = FlyControl.Para->POS_Outer_PID_z.Setpoint;		
	User_Data.Data2 = FlyControl.Para->POS_Outer_PID_z.Feedback;	
	User_Data.Data3 = FlyControl.Para->POS_Inner_PID_z.Setpoint;	
	User_Data.Data4 = FlyControl.Para->POS_Inner_PID_z.Feedback;			
	User_Data.Data5 = FlyControl.Para->POS_Acc_PID_z.Setpoint;
	User_Data.Data6 = FlyControl.Para->POS_Acc_PID_z.Feedback;;
	User_Data.Data7 = FlyControl.Para->Throttle;
	User_Data.Data8 = FlyControl.Para->POS_Acc_PID_z.Output;
	User_Data.Data9 = FlyControl.Para->POS_Acc_PID_z.SumError;
	User_Data.Data10 = 0;
	User_Data.Data11 = 0;
	User_Data.Data12 = 0;
	
//	User_Data.Data1 = Degrees(FlyControl.Para->ATT_Inner_PID_z.Setpoint);		
//	User_Data.Data2 = Degrees(Attitude.Rate->z);	
//	User_Data.Data3 = Degrees(FlyControl.Para->ATT_Outer_PID_z.Setpoint);		
//	User_Data.Data4 = Degrees(FlyControl.Para->ATT_Outer_PID_z.Feedback);			
//	User_Data.Data5 = Motor.PWM->PWM1;
//	User_Data.Data6 = GXGGA_Data.UTC;
//	User_Data.Data7 = Motor.PWM->PWM3;
//	User_Data.Data8 = Degrees(FlyControl.Para->ATT_Inner_PID_z.Feedback);;
//	User_Data.Data9 = FlyControl.Para->ATT_Inner_PID_z.SumError;
//	User_Data.Data10 = 0;
//	User_Data.Data11 = 0;
//	User_Data.Data12 = 0;
	
	Communicate_BUF[Cnt++] = 0xAA;
	Communicate_BUF[Cnt++] = 0xAA;
	Communicate_BUF[Cnt++] = 0xF1;
	Communicate_BUF[Cnt++] = 0;
	
	Temp = User_Data.Data1 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data2 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data3 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data4 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data5 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data6 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data7 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data8 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data9 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data10 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data11 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Temp = User_Data.Data12 ;
	Communicate_BUF[Cnt++] = BYTE1(Temp);
	Communicate_BUF[Cnt++] = BYTE0(Temp);
	
	Communicate_BUF[4] = Cnt - 5;
	
	u8 Sum = 0;
	for(u8 i=1;i<Cnt;i++)
		Sum += Communicate_BUF[i];
	
	Communicate_BUF[Cnt++]=Sum;

	Communicate_BUF[0] = Cnt - 1;    //HID第一个字节 表示数据长度
	
	usart_pc.send(Communicate_BUF,Cnt);
	return True;
}

void Send_Reply_PC(u16 Sum,u8 Head)
{
	Communicate_BUF[0] = 0X07;                      //帧长
	Communicate_BUF[1] = 0XAA;
	Communicate_BUF[2] = 0XAA;
	Communicate_BUF[3] = 0XEF;
	Communicate_BUF[4] = 2;
	Communicate_BUF[5] = Head;
	
	Communicate_BUF[6] = Sum;	
	
	u8 sum = 0;
	for(u8 i=1;i<7;i++)
		sum += Communicate_BUF[i];
	
	Communicate_BUF[7]=sum;	
	
	usart_pc.send(Communicate_BUF,8);
}




void Data_Analysis_PC(void)
{
	#define PC_BUF_SIZE 32
	u8 Rx_buf[PC_BUF_SIZE];
	
	u8 i = 0;
	u8 Start = 0;
	u8 Length = 0;
	u8 Sum = 0;
	u8 Funtion = 0;

	if(	usart_pc.receive(Rx_buf,PC_BUF_SIZE) == True) 
	{
		for(i = 0;i < PC_BUF_SIZE; i++)
		{
			if(i == 31) return;
			if(Rx_buf[i] == 0XAA & Rx_buf[i+1] == 0XAF)
				break;
		}
		
		Length = Rx_buf[i + 3] + 4;
		Start = i;
		Funtion = Rx_buf[i + 2];
		
		for(i = 0;i < Length;i++)
			Sum += Rx_buf[Start + i];
		
		if(Sum == Rx_buf[Start + Length])
		{ 
		 switch (Funtion) 																      //帧功能分析
			{
				case 0X01:
					MPU6050.IsCalibrating = 1;
					break;
				case 0X02:
					static u8 PID_Group = 1;
					Send_PID_Para_PC(PID_Group++);
					if(PID_Group>3)
						PID_Group = 1;
					break;
				case 0X10:                                           //PID1 2用于描述外环参数
					FlyControl.Para->ATT_Inner_PID_x.Kp = (float)(Rx_buf[Start + 4] << 8 | Rx_buf[Start + 5]);
					FlyControl.Para->ATT_Inner_PID_x.Ki = (float)(Rx_buf[Start + 6] << 8 | Rx_buf[Start + 7]);
					FlyControl.Para->ATT_Inner_PID_x.Kd = (float)(Rx_buf[Start + 8] << 8 | Rx_buf[Start + 9]);

					FlyControl.Para->ATT_Inner_PID_y.Kp = (float)(Rx_buf[Start + 10] << 8 | Rx_buf[Start + 11]);
					FlyControl.Para->ATT_Inner_PID_y.Ki = (float)(Rx_buf[Start + 12] << 8 | Rx_buf[Start + 13]);
					FlyControl.Para->ATT_Inner_PID_y.Kd = (float)(Rx_buf[Start + 14] << 8 | Rx_buf[Start + 15]);
				
					FlyControl.Para->ATT_Inner_PID_z.Kp = (float)(Rx_buf[Start + 16] << 8 | Rx_buf[Start + 17]);
					FlyControl.Para->ATT_Inner_PID_z.Ki = (float)(Rx_buf[Start + 18] << 8 | Rx_buf[Start + 19]);
					FlyControl.Para->ATT_Inner_PID_z.Kd = (float)(Rx_buf[Start + 20] << 8 | Rx_buf[Start + 21]);		

					Send_Reply_PC(Sum,0X10);
				
					break;
				case 0X11:
					FlyControl.Para->ATT_Outer_PID_x.Kp = (float)(Rx_buf[Start + 4] << 8 | Rx_buf[Start + 5])/100;
					FlyControl.Para->ATT_Outer_PID_x.Ki = (float)(Rx_buf[Start + 6] << 8 | Rx_buf[Start + 7])/100;
					FlyControl.Para->ATT_Outer_PID_x.Kd = (float)(Rx_buf[Start + 8] << 8 | Rx_buf[Start + 9])/100;

					FlyControl.Para->ATT_Outer_PID_y.Kp = (float)(Rx_buf[Start + 10] << 8 | Rx_buf[Start + 11])/100;
					FlyControl.Para->ATT_Outer_PID_y.Ki = (float)(Rx_buf[Start + 12] << 8 | Rx_buf[Start + 13])/100;
					FlyControl.Para->ATT_Outer_PID_y.Kd = (float)(Rx_buf[Start + 14] << 8 | Rx_buf[Start + 15])/100;
				
					FlyControl.Para->ATT_Outer_PID_z.Kp = (float)(Rx_buf[Start + 16] << 8 | Rx_buf[Start + 17])/100;
					FlyControl.Para->ATT_Outer_PID_z.Ki = (float)(Rx_buf[Start + 18] << 8 | Rx_buf[Start + 19])/100;
					FlyControl.Para->ATT_Outer_PID_z.Kd = (float)(Rx_buf[Start + 20] << 8 | Rx_buf[Start + 21])/100;	
				
					Send_Reply_PC(Sum,0X11);
					break;
				case 0X12:                                       
				
					Send_Reply_PC(Sum,0X12);
				
				case 0X13:
					Send_Reply_PC(Sum,0X13);
					break;
				case 0X14:
					Send_Reply_PC(Sum,0X14);
					break;
				case 0X15:
					Send_Reply_PC(Sum,0X15);
					break;
				default:
					break;
			}
		}
	}
}

void Send_Data_PC(void)
{
	static u8 Cnt = 0;

	switch (1) 
	{
		case 0:
			Send_Eular_PC();
			break;
		case 1:
			Send_Senser_PC();
			break;
		case 2:
			Send_RC_PC();
			break;
		case 3:
			Send_Motor_PC();
			break;
		case 4:
			Send_UserData_PC();
			break;
		case 5:
		Send_Senser2_PC();
		break;
		default:
			break;
	}
}

u32 Char_2_Int(u8 *str)
{
	u32 Float_Int_temp = 0;
	u8 * Str_Add_temp = str;
	u16 i = 0;

	for(i = 0;Str_Add_temp[i] != '}'; i++)
	{
		if( i > 10 )return 0;
		Float_Int_temp = (Float_Int_temp + (Str_Add_temp[i] - '0'))*10;
	}
	Float_Int_temp = Float_Int_temp*0.1;
}

void Send_Data_Phone(void)
{		char Tx_buffer[50];
		u8 Cnt = 0;
	if(SystemTime.Now_MS() < 5000)
		return;
		Cnt = sprintf(Tx_buffer,"{B%d:%d:%d:%d:%d}$",
	(int)(FlyControl.Para->POS_Outer_PID_z.Setpoint - FlyControl.Para->Home.z),
	(int)(Position.Position_xyz.z - FlyControl.Para->Home.z),
	(int)Attitude.Angle->z,
	(int)FlyControl.Para->POS_Acc_PID_z.Setpoint,
	(int)Position.Acc.z);        //Position.Speed.z Position.Acc.z
	
//		Cnt = sprintf(Tx_buffer,"{B%d:%d:%d:%d:%d}$",
//	(int)GPS_Location.POS_X,
//	(int)GPS_Location.POS_Y,
//	(int)GPS_Location.POS_Z,
//	(int)GPS_Location.state,
//	(int)GPS_Location.SatNum);        //Position.Speed.z Position.Acc.z
	
	usart_pc.send((u8*)Tx_buffer,Cnt);
	
	Cnt = sprintf(Tx_buffer,"{A%d:%d:%d:%d}$",
	0,
	0,
	0,
	(int)Attitude.Angle->z);
	usart_pc.send((u8*)Tx_buffer,Cnt);
}

void Data_Analysis_Phone(void)
{
	#define Phone_BUF_SIZE 10
	u8 Rx_buf[Phone_BUF_SIZE];
	char Tx_buffer[100];
	u32 Rx_Temp = 0;
	u8 Cnt = 0;
	u8 i = 0;
	u8 Funtion = 0;
	
	
	if(usart_pc.receive(Rx_buf,3) == True && Rx_buf[0] == '{' && Rx_buf[2] == ':')
	{
		for(i = 3;i < Phone_BUF_SIZE; i++)
		{
			if(usart_pc.receive(&Rx_buf[i],1) == False)break;
			
			if(Rx_buf[i] == '}') 
			{
				Funtion = Rx_buf[1];
				switch (Funtion) 																      //帧功能分析
					{
						case '0':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Outer_PID_z.Kp = Rx_Temp;
							break;
						case '1':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Outer_PID_z.Ki = Rx_Temp;					
							break;
						case '2': 
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Outer_PID_z.Kd = Rx_Temp;					
							break;
						case '3':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Inner_PID_z.Kp = Rx_Temp;					
							break;
						case '4':  
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Inner_PID_z.Ki = Rx_Temp;					
							break;			
						case '5':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Inner_PID_z.Kd = Rx_Temp;					
							break;
						case '6':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Acc_PID_z.Kp = Rx_Temp;					
							break;
						case '7':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Acc_PID_z.Ki = Rx_Temp;					
							break;
						case '8':
							Rx_Temp = Char_2_Int(&Rx_buf[3]);
							FlyControl.Para->POS_Acc_PID_z.Kd = Rx_Temp;					
							break;
						case 'Q':
							if(Rx_buf[3] == 'P')
							{
									Cnt = sprintf(Tx_buffer,"{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
								(int)FlyControl.Para->POS_Outer_PID_z.Kp					,
								(int)FlyControl.Para->POS_Outer_PID_z.Ki					,
								(int)FlyControl.Para->POS_Outer_PID_z.Kd					,
								(int)FlyControl.Para->POS_Inner_PID_z.Kp					,
								(int)FlyControl.Para->POS_Inner_PID_z.Ki					,
								(int)FlyControl.Para->POS_Inner_PID_z.Kd					,
								(int)FlyControl.Para->POS_Acc_PID_z.Kp					,
								(int)FlyControl.Para->POS_Acc_PID_z.Ki					,
								(int)FlyControl.Para->POS_Acc_PID_z.Kd					);
							usart_pc.send((u8*)Tx_buffer,Cnt);
							}
							break;
		//				case '9':
		//					break;
		//				case '9':
		//					break;
						default:
							break;
					}
				}
			}
	}
}


void Vcan_Send_Data_PC(void)
{
	float Temp1 = 0;
	u8 Cnt = 0;
	
	Communicate_BUF[Cnt++] = 0X03;
	Communicate_BUF[Cnt++] = 0XFC;
	
	Temp1 = 10 ;	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);	
	
	Temp1 = Position.Position_xyz.z;
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);
	
	Temp1 = Attitude.Angle->z;	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);	
	
	Temp1 = 40;
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);

	Temp1 = 50;
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);

	Temp1 = 60;
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);
	
	Temp1 = 70; 
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);

	Temp1 = 80;
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE2(Temp1);	
	Communicate_BUF[Cnt++] = BYTE3(Temp1);
	
	Communicate_BUF[Cnt++] = 0XFC;	
	Communicate_BUF[Cnt++] = 0X03;
	
	usart_pc.send(Communicate_BUF,Cnt);
}

#define USE_PHONE 0
#define USE_VCAN 1
#define USE_ANTO 0

BOOL Communicate(void)
{
//	static u8 Time_Cnt = 0;
//	if(Time_Cnt++ != 6)
//	{
//		return;
//	}
//	Time_Cnt = 0;
#if GPS_PC// GPS未占用串口1进行调试，才可正常使用
	GPS_UART.GPS_Cof();
#endif	
	
#if USE_PHONE
	Data_Analysis_Phone();
	Send_Data_Phone();
#endif	
	
#if USE_VCAN
		Vcan_Send_Data_PC();
#endif		
	
#if USE_ANTO
		Send_Data_PC();
		Data_Analysis_PC();
#endif

	return True;
}
