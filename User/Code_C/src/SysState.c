#include "SysState.h"

void State_Updata(u16 Time);

struct SystemState_ SystemState = 
{
	State_Updata
};



void RC_State(u16 Time)
{
	static u16 LostCnt = 0;
	//失控判定
	if((PWM_In.POS_Judge(PWM_RC_SW5) == PWM_In_Down) || PWM_In.Connect_Status == 0)
	{
		LostCnt += Time;
		if(LostCnt > 10)
		{
			FlyControl.Para->IsLost = True;
			LostCnt = 10;
		}
	}
	else
	{
		FlyControl.Para->IsLost = False;
		LostCnt = 0;	
	}
}

void Pow_State(u16 Time)
{
	BOOL IsAnyError = False;
	
	if(Power.Data->BAT_3S < 11.1f)
	{
		Led.Off(LED1);
		SYS_State.State1 |= POW_LOW;
	}
	else
	{
		SYS_State.State1 &= ~POW_LOW;	
	}
//	if(abs(Power.Data->BAT_12S - 50) > 20)        	IsAnyError = True;
//	if(abs(Power.Data->POW_4V5 - 4.5f) > 0.5f)      IsAnyError = True;
//	if(abs(Power.Data->POW_5V - 5.0f) > 0.5f)       IsAnyError = True;
//	if(abs(Power.Data->BAT_3S - 8.4f) > 3)          IsAnyError = True;
//	if(abs(Power.Data->Main_3V3_MCU - 3.3f) > 0.5f) IsAnyError = True;
//	if(abs(Power.Data->Main_3V3_SEN - 3.3f) > 0.5f) IsAnyError = True;
//	if(abs(Power.Data->Main_5V - 5.0f) > 0.5f)      IsAnyError = True;	
	
//	FlyControl.Para->IsError = IsAnyError;
//	
//	if(FlyControl.Para->IsError == True) SYS_State.State1 |= SYS_ERROR;
//	else SYS_State.State1 &= ~SYS_ERROR;
//	
//	if(Power.Data->IsUPS_On == True) SYS_State.State1 |= UPS_ON;
//	else SYS_State.State1 &= ~UPS_ON;
}


void Sensor_State(void)
{
	if(HMC5883.IsSensorError == True)
		SYS_State.State1 |= MAG_ERROR;
	else
		SYS_State.State1 &= ~MAG_ERROR;
}


void Controller_State(u16 Time)
{
	static u16 LockCnt = 0;
	static u16 UnlockCnt = 0;
	static u16 ModeCnt = 0;
	//控制模式判断
	if(PWM_RC_R6 < PWM_RC_DEAD)
	{
		if(FlyControl.Para->Mode != ATT && ModeCnt > 10)
		{
			FlyControl.Para->Mode = ATT;
			ModeCnt = 0;
		}
		else if(FlyControl.Para->Mode != ATT)
			ModeCnt++;
	}
	else
	{	
		if(FlyControl.Para->Mode != ALT && ModeCnt > 10)
		{
			FlyControl.Para->Mode = ALT;
			ModeCnt = 0;
		}
		else if(FlyControl.Para->Mode != ALT)
			ModeCnt++;
	}
	
	//解锁判定	 上锁判定
	if(FlyControl.Para->IsLock == True)// & (FlyControl.Para->IsError == False))
	{
		LockCnt = 0;
		if((PWM_RC_D_U < PWM_RC_MIN + 150) & (PWM_RC_Lr_Rr > PWM_RC_MAX - 150) & (FlyControl.Para->IsLost == False))
		{
			UnlockCnt += Time;
			if(UnlockCnt > 2000)	FlyControl.Para->IsLock = False;
		}
		else	UnlockCnt = 0;	
	} 
	else if(FlyControl.Para->IsLost == True)
		FlyControl.Para->IsLock = True;
	else
	{
		UnlockCnt = 0;
		if((PWM_RC_D_U < PWM_RC_MIN + 150) & (PWM_RC_Lr_Rr > PWM_RC_MAX - 150)) 
			LockCnt += Time;
		else
			LockCnt = 0;
		if(LockCnt > 2000)
			FlyControl.Para->IsLock = True;
	}
	
		//系统状态更新
	if(FlyControl.Para->IsLock == False) 
	{
		SYS_State.State1 |= UNLOCK;
		Led.Reverse(LED1,200);
	}
	else
	{
		SYS_State.State1 &= ~UNLOCK;
		Led.Reverse(LED1,500);
	}

	if(FlyControl.Para->IsLost != False) 
	{
		SYS_State.State1 |= LOST;
	}
	else
	{
		SYS_State.State1 &= ~LOST;
	}
}


void State_Updata(u16 Time)
{
	RC_State(Time);
	Controller_State(Time);
	Pow_State(Time);
//	Sensor_State();
}


