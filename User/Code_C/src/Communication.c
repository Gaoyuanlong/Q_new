#include "Communication.h"

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
	
	Temp1 = HMC5883.Data->MAG_ADC.x*100;	
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);		
	
	Temp1 = HMC5883.Data->MAG_ADC.y*100;
	Communicate_BUF[Cnt++] = BYTE1(Temp1);	
	Communicate_BUF[Cnt++] = BYTE0(Temp1);	
	
	Temp1 = HMC5883.Data->MAG_ADC.z*100;
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

void Send_Eular_PC(void)
{
	u16 Temp1 = 0;
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
	
	Communicate_BUF[Cnt++] = MPU6050.IsCalibrating;
	Communicate_BUF[Cnt++] = True;	

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
	

	User_Data.Data1 = Degrees(FlyControl.Para->ATT_Inner_PID_z.Setpoint);		
	User_Data.Data2 = Degrees(Attitude.Rate->z);	
	User_Data.Data3 = Degrees(FlyControl.Para->ATT_Outer_PID_z.Setpoint);		
	User_Data.Data4 = Degrees(FlyControl.Para->ATT_Outer_PID_z.Feedback);			
	User_Data.Data5 = Motor.PWM->PWM1;
	User_Data.Data6 = Motor.PWM->PWM2;
	User_Data.Data7 = Motor.PWM->PWM3;
	User_Data.Data8 = Degrees(FlyControl.Para->ATT_Inner_PID_z.Feedback);;
	User_Data.Data9 = FlyControl.Para->ATT_Inner_PID_z.SumError;
	User_Data.Data10 = 0;
	User_Data.Data11 = 0;
	User_Data.Data12 = 0;
	
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


void Send_Data_PC(void)
{
	static u8 Cnt = 0;

	switch (Cnt++%5) 
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
		default:
			break;
	}
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


BOOL Communicate(void)
{
#ifndef ENABLE_GPS_PC// GPS未占用串口1进行调试，才可正常使用
	
	
	Send_Data_PC();
	Data_Analysis_PC();
	
	
#else
	GPS_UART.GPS_Cof();
#endif
	return True;
}
