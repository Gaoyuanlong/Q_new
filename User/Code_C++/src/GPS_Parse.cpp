#include "GPS_Parse.h"
#include "GPS_UART.h"
#include "arm_math.h"
#include "Math_User.h"
#include "Attitude.h"

double GPS_Str2Float(const u8* Str_Add)
{ 
	double Float_Int_temp = 0;
	double Float_Float_temp = 0;
	const u8* Str_Add_temp = Str_Add;
	u8 i = 0 , j = 0;
	
	if(Str_Add_temp[i] == '-')
	{
		i++;
	}
	for(; Str_Add_temp[i] != '.' && Str_Add_temp[i] != ','; i++)
	{
		if( i > 10 || Str_Add_temp[i] == '*')
			return 0;
		Float_Int_temp = (Float_Int_temp + (Str_Add_temp[i] - '0'))*10;

	}
	Float_Int_temp = Float_Int_temp*0.1;
	if(Str_Add_temp[i] == '.')
		i++ ;
	
	for(; Str_Add_temp[i] != ',' ; i++)
	{
		if( i > 20 || Str_Add_temp[i] == '*')
			return 0;
		Float_Float_temp = (Float_Float_temp + (Str_Add_temp[i] - '0'))*10;
		j++;
	}
	
	for(j++;j>0;j--)
	{
		Float_Float_temp = Float_Float_temp*0.1;
	}
	
	if(Str_Add_temp[0] == '-')
		return -(Float_Int_temp + Float_Float_temp);
	else
		return (Float_Int_temp + Float_Float_temp);
}

//-------GXGGA数据解析---------------------------------------//
void GXGGA::GXGGA_Data_Clear(void)
{
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
}	
void GPS::GPS_Parse(GXGGA &GXGGA_Data ,const u8* BUF)
{
	#define ONE_DATA_MAX_SIZE 20
	const u8* Str_Add_temp = BUF;
	u16 i = 0;
	u8 j = 0;
	
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      	//1 UTC 时间
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.UTC = 0;
	else GXGGA_Data.UTC = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //2 纬度
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.LAT = 0;
	else GXGGA_Data.LAT = GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //3 N 或 S
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.NorS = 0;
	else GXGGA_Data.NorS = Str_Add_temp[i];		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //4 经度
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.LON = 0;
	else GXGGA_Data.LON = GPS_Str2Float(&Str_Add_temp[i]);	
		 
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //5 E 或 W
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.WorE = 0;
	else GXGGA_Data.WorE = Str_Add_temp[i];		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //6 定位质量指示，0=定位无效，1=定位有效
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.state = 0;
	else GXGGA_Data.state = Str_Add_temp[i] - '0';
				
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //7 使用卫星数量，从 00 到 12
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.SatNum = 0;
	else GXGGA_Data.SatNum = (u8)GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //8 水平精确度，0.5 到 99.9
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.HDOP = 0;
	else GXGGA_Data.HDOP = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //9 天线离海平面的高度，-9999.9 到 9999.9 米
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.AltSea = 0;
	else GXGGA_Data.AltSea = GPS_Str2Float(&Str_Add_temp[i]);	
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      // 单位M 不处理
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //10 大地水准面高度，-9999.9 到 9999.9 米
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.AltEarth = 0;
	else GXGGA_Data.AltEarth = GPS_Str2Float(&Str_Add_temp[i]);
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      // 单位M 不处理
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //11 差分 GPS 数据期限
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.GPSBaseTime = 0;
	else GXGGA_Data.GPSBaseTime = (u16)GPS_Str2Float(&Str_Add_temp[i]);

	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 差分参考基站标号
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.GPSBase = 0;
	else GXGGA_Data.GPSBase = (u16)GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  '*'){if(j++ > ONE_DATA_MAX_SIZE)return;}      //13 语句结束标志符

}
//-------GXRMC 数据解析---------------------------------------//
void GXRMC::GXRMC_Data_Clear(void)
{
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
}
void GPS::GPS_Parse(GXRMC &GXRMC_Data ,const u8* BUF)
{
	#define ONE_DATA_MAX_SIZE 20
	const u8* Str_Add_temp = BUF;
	u16 i = 0;
	u8 j = 0;
	
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      	//1 UTC 时间
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.UTC = 0;
	else GXRMC_Data.UTC = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //2 定位状态 0：未定位 1：无差分，SPS模式 2：带差分，SPS模式 3：PPS模式
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.state = 0;
	else GXRMC_Data.state = Str_Add_temp[i];	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //3 纬度
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.LAT = 0;
	else GXRMC_Data.LAT = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //4  N 或 S
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.NorS = 0;
	else GXRMC_Data.NorS = Str_Add_temp[i];	
		 
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //5 经度
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.LON = 0;
	else GXRMC_Data.LON = GPS_Str2Float(&Str_Add_temp[i]);				
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //6 E 或 W
	if(Str_Add_temp[i] ==  ',')  GXRMC_Data.WorE = 0;
	else GXRMC_Data.WorE = Str_Add_temp[i];
				
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //7 地面速率（000.0~999.9节，前面的0也将被传输）
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.SOG = 0;
	else GXRMC_Data.SOG = GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //8 地面航向（000.0~359.9度，以真北为参考基准，前面的0也将被传输）
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.COG = 0;
	else GXRMC_Data.COG = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //9 日期 ddmmyy（日月年）格式
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Date = 0;
	else GXRMC_Data.Date = GPS_Str2Float(&Str_Add_temp[i]);	
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //10 磁偏角 （000.0~180.0度，前面的0也将被传输）
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mag_Var = 0;
	else GXRMC_Data.Mag_Var = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 磁偏角方向 E（东）或W（西）
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mag_EorW = 0;
	else GXRMC_Data.Mag_EorW = Str_Add_temp[i];
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 模式指示（仅NMEA0183 3.00版本输出，A=自主定位，D=差分，E=估算，N=数据无效）
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mode = 0;
	else GXRMC_Data.Mode = Str_Add_temp[i];
		
	j = 0;
	while(Str_Add_temp[i++] !=  '*'){if(j++ > ONE_DATA_MAX_SIZE)return;}      //13 语句结束标志符
}
//-------GXGLL 数据解析---------------------------------------//
void GPS::GPS_Parse(GXGLL &GXGLL_Data ,const u8* BUF)
{

}
//-------找帧头---------------------------------------//
u8 GPS::GPS_Find_Head(const char* Str1,const char* Str2,const char* Str3)
{
	u8 BUF = 0;
	u8 i = 0;
	const char* P1 = Str1;
	const char* P2 = Str2;
	const char* P3 = Str3;
	
	while((i++)<GPS_DATA_MAX_SIZE)
	{
		if(False == GPS_UART.receive(&BUF,1))return 0;
		
		if(BUF == *P1)
			P1++;
		else
			P1 = Str1;
		
		if(BUF == *P2)
			P2++;
		else
			P2 = Str2;
		
	  if(BUF == *P3)
			P3++;
		else
			P3 = Str3;
		
		if(*P1 == '\0')
			return 1;
		if(*P2 == '\0')
			return 2;
		if(*P3 == '\0')
			return 3;
	}
	
	return 0;
}

BOOL GPS::GPS_Read_Str(u8 *data, u16 num)
{
	u8 BUF = 0;
	static u8 i = 0;
	while(i < GPS_DATA_MAX_SIZE)
	{
		if(False == GPS_UART.receive(&BUF,1))return False;
		
		data[i] = BUF;
		if(data[i] == '*')
		{
			i = 0;
			return True;
		}
		i++;
	}
	i = 0;
	return False;
}


GXGGA GXGGA_Data;
GXRMC GXRMC_Data;
GPS GPS_Location;
void GPS::GPS_Update(void)
{
	static u8 Time_10 = 0;
	static u8 Temp = 0;
	static u8 Gps_original_data[GPS_DATA_MAX_SIZE+1] ={0};
	
	if(Time_10++ > 1)
	{
		if( Temp == 0)
			Temp = GPS_Find_Head("$GNGGA","$GNRMC","----");
		switch(Temp)
		{
			case 1:
				if(GPS_Read_Str(Gps_original_data,GPS_DATA_MAX_SIZE) == False)break;
				//GXGGA_Data.GXGGA_Data_Clear();
				GPS_Location.GPS_Parse(GXGGA_Data,Gps_original_data);
				GPS_Unit_transform();
				Temp = 0;
			break;
			case 2:
				if(GPS_Read_Str(Gps_original_data,GPS_DATA_MAX_SIZE) == False)break;
				//GXRMC_Data.GXRMC_Data_Clear();
				GPS_Location.GPS_Parse(GXRMC_Data,Gps_original_data);
				GPS_Unit_transform();
				Temp = 0;
			break;
			case 3:

			break;
			default:break;
		}
		Time_10 = 0;
	}
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
*/
void GPS::GPS_Unit_transform(void)
{
	#define EARTH_RADIUS 6371393
	#define LAT0 0
	#define LON0 0
	static u8 Cnt = 0;
	double  Lon_Deg = 0 ,Lat_Deg = 0;
	int Lat_Int_Temp = 0, Lon_Int_Temp = 0;
	
	Lon_Int_Temp = (int)GXGGA_Data.LON/100; 
	Lon_Deg = Lon_Int_Temp + (GXGGA_Data.LON - Lon_Int_Temp*100)/60;
	
	Lat_Int_Temp = (int)GXGGA_Data.LAT/100; 
	Lat_Deg = Lat_Int_Temp + (GXGGA_Data.LAT - Lat_Int_Temp*100)/60;
	
	if(GXGGA_Data.WorE == 'W')
		Lon_Deg = -Lon_Deg;
	if(GXGGA_Data.NorS == 'S')
		Lat_Deg = -Lat_Deg;
	
	POS_X = (Lon_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectX;		//经度 单位cm	GGA
	POS_Y = (Lat_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectY;		//纬度 单位cm	GGA
	POS_Z = GXGGA_Data.AltSea * 100 - Home_OffectZ; 								//相对海平面高度 单位cm	GGA
	//位置需要加入滤波 备忘
	float Speed_Temp = GXRMC_Data.SOG * 51.4; 			//速率 单位cm/s   节转cm/s
	float Angle_Temp = GXRMC_Data.COG * DEG_TO_RAD;	//航向角 0-360 		弧度转度
	Speed.x = Speed_Temp * arm_sin_f32(Angle_Temp);	// 单位cm/s
	Speed.y = Speed_Temp * arm_cos_f32(Angle_Temp);	// 单位cm/s
	//速度需要加入滤波	备忘
	SatNum = GXGGA_Data.SatNum;
	state = GXGGA_Data.state;
	
	//位置偏置取值  暂时使用 后期需要修改
	if(state != 0 && Cnt < 105)
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




