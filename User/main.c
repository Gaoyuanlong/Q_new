#include "Board.h"

Target_ TG_50HZ(Scheduler.Loop_50Hz,20000);
Target_ TG_200HZ(Scheduler.Loop_200Hz,5000);
Target_ TG_500HZ(Scheduler.Loop_500Hz,2000);

extern void TIM5_CH1_Cap_Init(u32 arr,u16 psc);
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
	SystemTime.Init();
	Led.Init();
	Buzzer.Init();
	usart1.init(115200);
	Power.Init();
	MPU6050.Init();
	MS5611.Init();
	HMC5883.Init();
	Motor.Init();
//GPS.Init();
	PWM_In.Init(2000,4000);
//SD_APP.init();
//Altitude.Init();
	while(1)
	{		
		TG_50HZ.Run();
		TG_200HZ.Run();
		TG_500HZ.Run();
	}
}



/*
Œ Ã‚£∫
	 ±÷”≤Ó2us


*/



