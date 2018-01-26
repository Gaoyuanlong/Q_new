#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
#include "stm32f4xx.h"
#include "Common.h"
#define GPS_DATA_MAX_SIZE 100 //GPS帧最大字符数


class GXGGA
{
	public:
		GXGGA(){
		 UTC = 0;          //UTC时间
		 LAT = 0;          //纬度
		 NorS = 0;         //南北
		 LON = 0;          //经度
		 WorE = 0;          //东西
		 state = 0;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
		 SatNum = 0;          //卫星数量
		 HDOP = 0;         //水平经度衰减因子
		 AltSea = 0;       //海平面高度
		 AltEarth = 0;     //相对于海平面的地面高度
		 GPSBaseTime = 0;    //上一次收到GPS差分站数据开始的计时，单位秒
		 GPSBase = 0;        //差分站编号
		};
	
		double UTC;          //UTC时间
		double LAT;          //纬度
		char  NorS;         //南北
		double LON;          //经度
		char WorE;          //东西
		u8 state;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
		u8 SatNum;          //卫星数量
		double HDOP;         //水平经度衰减因子
		double AltSea;       //海平面高度
		double AltEarth;     //相对于海平面的地面高度
		u16 GPSBaseTime;    //上一次收到GPS差分站数据开始的计时，单位秒
		u16 GPSBase;        //差分站编号
		
		void GXGGA_Data_Clear(void);
		private:
};
class GXRMC
{
		public:
		GXRMC(){
		 UTC = 0;         //UTC时间
		 state = 0;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
		 LAT = 0;         //纬度
		 NorS = 0;         //南北
		 LON = 0;         //经度
		 WorE = 0;          //东西
		 SOG = 0;  				//地面速率（000.0~999.9节，前面的0也将被传输）
		 COG = 0;					//地面航向（000.0~359.9度，以真北为参考基准，前面的0也将被传输）
		 Date = 0;        //日期 ddmmyy（日月年）格式
		 Mag_Var = 0; 		//磁偏角 （000.0~180.0度，前面的0也将被传输）
		 Mag_EorW = 0;			//磁偏角方向 E（东）或W（西）
		 Mode = 0;  				//模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）
		};
	
		double UTC;         //UTC时间
		u8 state;           //定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
		double LAT;         //纬度
		char  NorS;         //南北
		double LON;         //经度
		char WorE;          //东西
		double SOG;  				//地面速率（000.0~999.9节，前面的0也将被传输）
		double COG;					//地面航向（000.0~359.9度，以真北为参考基准，前面的0也将被传输）
		double Date;        //日期 ddmmyy（日月年）格式
		double Mag_Var; 		//磁偏角 （000.0~180.0度，前面的0也将被传输）
		char Mag_EorW;			//磁偏角方向 E（东）或W（西）
		char Mode;  				//模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）

		void GXRMC_Data_Clear(void);
		private:
	
};

class GXGLL
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
	double POS_X;
	double POS_Y;
	double POS_Z;
	void GPS_Update(void);
			
	private:
		u8 GPS_Find_Head(const char* Str1,const char* Str2,const char* Str3);
		BOOL GPS_Read_Str(u8 *data, u16 num);
		void GPS_Parse(GXGGA &GXGGA_Data ,const u8* BUF);	
		void GPS_Parse(GXGLL &GXGLL_Data ,const u8* BUF);
	  void GPS_Parse(GXRMC &GXGLL_Data ,const u8* BUF);
		void GPS_Unit_transform(void);
};

extern GPS GPS_Location;
extern GXGGA GXGGA_Data;
extern GXRMC GXRMC_Data;
#endif


