#include "GPS_Parse.h"
#include "GPS_UART.h"
#include "arm_math.h"
#include "Math_User.h"
#include "Attitude.h"

GPS_PVT GPS_PVT_Data;
void GPS_PVT::GPS_PVT_Parse(void)
{
	u8 i = 0;
	u8 Start = 0;
	u16 Length = 0;
	u8 CK_A = 0;
	u8 CK_B = 0;
	
	if(GPS_UART.receive(Rx_buf,GPS_PVT_BUF_SIZE) != 0) 
	{
		for(i = 0;i < GPS_PVT_BUF_SIZE; i++)
		{
			if((GPS_PVT_BUF_SIZE - 1) == i)
				return;
			if(Rx_buf[i] == 0XB5 & Rx_buf[i+1] == 0X62)
				break;
		}
		
		Length = (u16)Rx_buf[i + 4] + ((u16)Rx_buf[i + 5]<<8);
		if(i + Length + 7 > GPS_PVT_BUF_SIZE)
			return;
		

		Start = i + 2;
		for(i = 0;i < 4 + Length;i++)
		{
			CK_A = CK_A + Rx_buf[Start + i];
			CK_B = CK_B + CK_A;		
		}	
		if(CK_A == Rx_buf[Start + 4 + Length] && CK_B == Rx_buf[Start + 5 + Length])
		{ 
			PVT_Data.iTOW 				= ((PVT_Data_*)&Rx_buf[Start + 4])->iTOW;
			PVT_Data.year 				= ((PVT_Data_*)&Rx_buf[Start + 4])->year;
			PVT_Data.month 				= ((PVT_Data_*)&Rx_buf[Start + 4])->month;
			PVT_Data.day 					= ((PVT_Data_*)&Rx_buf[Start + 4])->day;
			PVT_Data.hour 				= ((PVT_Data_*)&Rx_buf[Start + 4])->hour;
			PVT_Data.min 					= ((PVT_Data_*)&Rx_buf[Start + 4])->min;
			PVT_Data.sec 					= ((PVT_Data_*)&Rx_buf[Start + 4])->sec;
			PVT_Data.valid 				= ((PVT_Data_*)&Rx_buf[Start + 4])->valid;
			PVT_Data.tAcc					= ((PVT_Data_*)&Rx_buf[Start + 4])->tAcc;
			PVT_Data.nano 				= ((PVT_Data_*)&Rx_buf[Start + 4])->nano;
			PVT_Data.fixType 			= ((PVT_Data_*)&Rx_buf[Start + 4])->fixType;
			PVT_Data.flags 				= ((PVT_Data_*)&Rx_buf[Start + 4])->flags;
			PVT_Data.reserved1 		= ((PVT_Data_*)&Rx_buf[Start + 4])->reserved1;
			PVT_Data.numSV 				= ((PVT_Data_*)&Rx_buf[Start + 4])->numSV;
			PVT_Data.lon 					= ((PVT_Data_*)&Rx_buf[Start + 4])->lon;
			PVT_Data.lat 					= ((PVT_Data_*)&Rx_buf[Start + 4])->lat;
			PVT_Data.height 			= ((PVT_Data_*)&Rx_buf[Start + 4])->height;
			PVT_Data.hMSL 				= ((PVT_Data_*)&Rx_buf[Start + 4])->hMSL;
			PVT_Data.hAcc 				= ((PVT_Data_*)&Rx_buf[Start + 4])->hAcc;
			PVT_Data.hAcc 				= ((PVT_Data_*)&Rx_buf[Start + 4])->hAcc;
			PVT_Data.velN 				= ((PVT_Data_*)&Rx_buf[Start + 4])->velN;
			PVT_Data.velE 				= ((PVT_Data_*)&Rx_buf[Start + 4])->velE;
			PVT_Data.velD 				= ((PVT_Data_*)&Rx_buf[Start + 4])->velD;
			PVT_Data.gSpeed 			= ((PVT_Data_*)&Rx_buf[Start + 4])->gSpeed;
			PVT_Data.heading 			= ((PVT_Data_*)&Rx_buf[Start + 4])->heading;
			PVT_Data.sAcc 				= ((PVT_Data_*)&Rx_buf[Start + 4])->sAcc;
			PVT_Data.headingAcc 	= ((PVT_Data_*)&Rx_buf[Start + 4])->headingAcc;
			PVT_Data.pDOP 				= ((PVT_Data_*)&Rx_buf[Start + 4])->pDOP;
			PVT_Data.reserved2 		= ((PVT_Data_*)&Rx_buf[Start + 4])->reserved2;
			PVT_Data.reserved3 		= ((PVT_Data_*)&Rx_buf[Start + 4])->reserved3;
		}
	}	
}

GPS GPS_Location;
void GPS::GPS_Update(void)
{
	GPS_PVT_Data.GPS_PVT_Parse();
	GPS_Unit_transform();
}

/*
		设第一点A的经 纬度为(LonA, LatA)，第二点B的经纬度为(LonB, LatB)，
		按照0度经线的基准，东经取经度的正值(Longitude)，西经取经度负值(-Longitude)，
		北纬取90-纬度值(90- Latitude)，南纬取90+纬度值(90+Latitude)，
		则经过上述处理过后的两点被计为(MLonA, MLatA)和(MLonB, MLatB)。
		那么根据三角推导，可以得到计算两点距离的如下公式：
		C = sin(MLatA)*sin(MLatB)*cos(MLonA-MLonB) + cos(MLatA)*cos(MLatB)
		Distance = R*Arccos(C)*Pi/180

		度分转换：
		将度分单位数据转换为度单位数据 
		度=度+分/60
		例如：
		经度 = 116°20.12’
		纬度 = 39°12.34’ 
		经度 = 116 + 20.12 / 60 = 116.33533° 
		纬度 = 39 + 12.34 / 60 = 39.20567°

		弧长公式
		l = n（圆心角）× π（圆周率）× r（半径）/180=α(圆心角弧度数)× r（半径）
		
		直接转换：
		地球子午线（南极到北极的连线）长度39940.67公里，纬度一度合110.94公里，一分合1.849公里，一秒合30.8米，不同纬度的间距是一样的。
		地球赤道圈长度40075.36公里，北京和乌鲁木齐地区在北纬40度左右，纬度圈长为40075*sin(90-40)，因此这里的经度一度合85.276公里，一分合1.42公里，一秒合23.69米。
		在除赤道外的其他纬线上，经度差1度对应的实际距离是 110.94公里*cos纬度 。
*/
void GPS::GPS_Unit_transform(void)
{
	#define EARTH_RADIUS 6371393
	#define LAT0 0
	#define LON0 0
	static u8 Cnt = 0;
	static double Lon_Deg = 0 ,Lat_Deg = 0;
	int Lat_Int_Temp = 0, Lon_Int_Temp = 0;
	
//	Lon_Int_Temp = (int)GPS_PVT_Data.PVT_Data.lon/100; 
//	Lon_Deg = Lon_Int_Temp + (GPS_PVT_Data.PVT_Data.lon - Lon_Int_Temp*100)/60;
//	
//	Lat_Int_Temp = (int)GPS_PVT_Data.PVT_Data.lat/100; 
//	Lat_Deg = Lat_Int_Temp + (GPS_PVT_Data.PVT_Data.lat - Lat_Int_Temp*100)/60;
	
//	if(GXGGA_Data.WorE == 'W')
//		Lon_Deg = -Lon_Deg;
//	if(GXGGA_Data.NorS == 'S')
//		Lat_Deg = -Lat_Deg;
//	POS_X = (Lon_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectX;		//经度 单位cm
//	POS_Y = (Lat_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectY;		//纬度 单位cm
//	POS_Z = GPS_PVT_Data.PVT_Data.hMSL * 10 - Home_OffectZ; 								//相对海平面高度 单位cm	

	Lon_Deg = GPS_PVT_Data.PVT_Data.lon;
	Lat_Deg = GPS_PVT_Data.PVT_Data.lat;
	
	
//	POS_X = Lon_Deg * cos(Lat_Deg*DEG_TO_RAD) * 11094000 - Home_OffectX;					//经度 单位cm
//	POS_Y = Lat_Deg * 11094000 - Home_OffectY;																		//纬度 单位cm

	POS_X = Lon_Deg - Home_OffectX;					//经度 单位度 *1e^-7
	POS_Y = Lat_Deg - Home_OffectY;					//纬度 单位度 *1e^-7
	POS_Z = GPS_PVT_Data.PVT_Data.hMSL * 10 - Home_OffectZ; 											//相对海平面高度 单位cm	

	//位置需要加入滤波 备忘
	Speed.x = GPS_PVT_Data.PVT_Data.velE * 0.1;	// 单位cm/s
	Speed.y = GPS_PVT_Data.PVT_Data.velN * 0.1;	// 单位cm/s
	Speed.z = GPS_PVT_Data.PVT_Data.velD * 0.1;	// 单位cm/s
	//速度需要加入滤波	备忘
	state = GPS_PVT_Data.PVT_Data.flags&0x01;
	SatNum = GPS_PVT_Data.PVT_Data.numSV;
	pDOP = GPS_PVT_Data.PVT_Data.pDOP*0.01;
	
	//位置偏置取值  暂时使用 后期需要修改
	if(state!= 0 && Cnt < 105)
	{
		Cnt++;
	}
	if(Cnt == 100)
	{
		Home_OffectX = POS_X;
		Home_OffectY = POS_Y;
		Home_OffectZ = POS_Z;
	}
}




