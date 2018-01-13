#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
#include "stm32f4xx.h"
#include "Common.h"
#define GPS_DATA_MAX_SIZE 250 //GPS帧最大字符数


class GXGGA
{
	public:
		GXGGA(){};
	
		float UTC;          //UTC时间
		float LAT;          //纬度
		char  NorS;         //南北
		float LON;          //经度
		char WorE;          //东西
		u8 state;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
		u8 SatNum;          //卫星数量
		float HDOP;         //水平经度衰减因子
		float AltSea;       //海平面高度
		float AltEarth;     //相对于海平面的地面高度
		u16 GPSBaseTime;    //上一次收到GPS差分站数据开始的计时，单位秒
		u16 GPSBase;        //差分站编号
			
		private:
};

class GXGLL
{
};

class GXRMC
{
};
class GXVTG
{
};



class GPS
{
	public:
		GPS(){};
			
	u8 state;
	u8 SatNum;
	float POS_X;
	float POS_Y;
	float POS_Z;
	void GPS_Update(void);
			
	private:
		u8 GPS_Find_Head(const char* Str1,const char* Str2,const char* Str3);
		void GPS_Parse(GXGGA &GXGGA_Data ,const u8* BUF);	
		void GPS_Parse(GXGLL &GXGLL_Data ,const u8* BUF);
	  void GPS_Parse(GXRMC &GXGLL_Data ,const u8* BUF);
		void GPS_Unit_transform(void);
};

extern GPS GPS_Location;
#endif


