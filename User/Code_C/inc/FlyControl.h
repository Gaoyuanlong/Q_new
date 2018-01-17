#ifndef __FLYCONTROL__H
#define __FLYCONTROL__H
#include "PID.h"
#include "Attitude.h"
#include "Common.h"
#include "SBUS.h"
#include "Motor.h"
#include "HMC5883.h"
#include "PWM_Capture.h"

#define THROTTLE_MIN PWM_RC_MIN
#define THROTTLE_MAX PWM_RC_MAX
#define THROTTLE_MID ((PWM_RC_MAX + PWM_RC_MIN) / 2)
#define THROTTLE_40_PERCENT ((THROTTLE_MAX - THROTTLE_MIN) * 0.4f)

enum Fly_Mode
{
	ATT,
	ALT,
	POS,
};

struct Control_Para_
{
	PID ATT_Inner_PID_x;
	PID ATT_Inner_PID_y;
	PID ATT_Inner_PID_z;
	
	PID ATT_Outer_PID_x;
	PID ATT_Outer_PID_y;	
	PID ATT_Outer_PID_z;	
	
	PID POS_Acc_PID_x;
	PID POS_Acc_PID_y;
	PID POS_Acc_PID_z;
	
	PID POS_Inner_PID_x;
	PID POS_Inner_PID_y;
	PID POS_Inner_PID_z;
	
	PID POS_Outer_PID_x;
	PID POS_Outer_PID_y;
	PID POS_Outer_PID_z;
	int Throttle;

	Vector Home;
	BOOL IsLock;
	BOOL IsLost;
	BOOL IsError;
	Fly_Mode Mode;
};

extern struct FlyControl_
{
	struct Control_Para_* Para;

	void (*ATT_InnerLoop)(u32 Time);
	void (*ATT_OuterLoop)(u32 Time);
	
	void (*POS_AccLoop)(u32 Time);
	void (*POS_InnerLoop)(u32 Time);
	void (*POS_OuterLoop)(u32 Time);
	
}FlyControl;

#endif
