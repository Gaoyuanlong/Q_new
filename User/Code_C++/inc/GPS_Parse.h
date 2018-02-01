#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
#include "stm32f4xx.h"
#include "Common.h"

class PVT_Data_
{
	public:
	u32 iTOW;
	u16 year;
	u8 month;
	u8 day;
	u8 hour;
	u8 min;
	u8 sec;
	u8 valid;
	u32 tAcc;
	s32 nano;
	u8 fixType;
	u8 flags;
	u8 reserved1;
	u8 numSV;
	s32 lon;
	s32 lat;
	s32 height;
	s32 hMSL;
	u32 hAcc;
	u32 vAcc;
	s32 velN;
	s32 velE;
	s32 velD;
	s32 gSpeed;
	s32 heading;
	u32 sAcc;
	u32 headingAcc;
	u16 pDOP;
	u16 reserved2;
	u32 reserved3;
};
#define GPS_PVT_BUF_SIZE 110
class GPS_PVT
{
public:
	GPS_PVT()
	{}
	void GPS_PVT_Parse(void);
private:
	u8 Rx_buf[GPS_PVT_BUF_SIZE];
	PVT_Data_ PVT_Data;
};

class GPS
{
	public:
		GPS(){
		Home_OffectX = 0;
		Home_OffectY = 0;
		Home_OffectZ = 0;
		};
			
	u8 state;
	u8 SatNum;
	double Home_OffectX;
	double Home_OffectY;
	double Home_OffectZ;
			
	double POS_X;
	double POS_Y;
	double POS_Z;
	Vector Speed;
	void GPS_Update(void);
			
	private:
		void GPS_Unit_transform(void);
};

extern GPS GPS_Location;
#endif


