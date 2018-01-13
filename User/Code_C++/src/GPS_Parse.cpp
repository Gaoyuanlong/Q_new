#include "GPS_Parse.h"
#include "GPS_UART.h"

double RTK_Str2Float(const u8* Str_Add)
{ 
	double Float_Int_temp = 0;
	double Float_Float_temp = 0;
	const u8* Str_Add_temp = Str_Add;
	u8 i = 0 , j = 0;
	
	if(Str_Add_temp[i] == '-')
	{
		i++;
	}
	for(; Str_Add_temp[i] != '.' && Str_Add_temp[i] != ',' ; i++)
	{
		Float_Int_temp = (Float_Int_temp + (Str_Add_temp[i] - '0'))*10;
	}
	Float_Int_temp = Float_Int_temp*0.1;
	if(Str_Add_temp[i] == '.')
		i++ ;
	
	for(; Str_Add_temp[i] != ',' ; i++)
	{
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
void GPS::GPS_Parse(GXGGA &GXGGA_Data ,const u8* BUF)
{
	const u8* Str_Add_temp = BUF;
	u8 i = 0;
	
	while(Str_Add_temp[i++] != ','){if(i > GPS_DATA_MAX_SIZE)return;}      	//1
	GXGGA_Data.UTC = RTK_Str2Float(&Str_Add_temp[i]);		
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //2
	GXGGA_Data.LAT = RTK_Str2Float(&Str_Add_temp[i]);
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //3
	GXGGA_Data.NorS = Str_Add_temp[i];		
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //4
	GXGGA_Data.LON = RTK_Str2Float(&Str_Add_temp[i]);	
		 
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //5
	GXGGA_Data.WorE = Str_Add_temp[i];		
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //6
	GXGGA_Data.state = Str_Add_temp[i];
				
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //7
	GXGGA_Data.SatNum = (u8)RTK_Str2Float(&Str_Add_temp[i]);
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //8
	GXGGA_Data.HDOP = RTK_Str2Float(&Str_Add_temp[i]);	
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //9
	GXGGA_Data.AltSea = RTK_Str2Float(&Str_Add_temp[i]);	
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //10
	GXGGA_Data.AltEarth = Str_Add_temp[i];

	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //11
	GXGGA_Data.GPSBaseTime = RTK_Str2Float(&Str_Add_temp[i]);

	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //12
	GXGGA_Data.GPSBaseTime = Str_Add_temp[i];	
		
	while(Str_Add_temp[i++] !=  ','){if(i > GPS_DATA_MAX_SIZE)return;}      //13
	GXGGA_Data.GPSBase = (u16)RTK_Str2Float(&Str_Add_temp[i]);	
		
	while(Str_Add_temp[i++] !=  '*'){if(i > GPS_DATA_MAX_SIZE)return;}      //14

}
//-------GXGLL 数据解析---------------------------------------//
void GPS::GPS_Parse(GXGLL &GXGLL_Data ,const u8* BUF)
{

}
//-------GXRMC 数据解析---------------------------------------//
void GPS::GPS_Parse(GXRMC &GXGLL_Data ,const u8* BUF)
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
		GPS_UART.receive(&BUF,1);
		
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
		
		if(*P1 == '\0')return 1;
		if(*P2 == '\0')return 2;
		if(*P3 == '\0')return 3;
	}
	
	return 0;
}


GPS GPS_Location;
GXGGA GXGGA_Data;
GXGLL GXGLL_Data;
GXRMC GXRMC_Data;
void GPS::GPS_Update(void)
{
	u8 Temp = 0;
	u8 Gps_original_data[GPS_DATA_MAX_SIZE];
	
	Temp = GPS_Find_Head("$GPGGA","$GPGGA","$GPGGA");
  GPS_UART.receive(Gps_original_data,GPS_DATA_MAX_SIZE);
	
	switch(Temp)
	{
		case 1:
			GPS_Location.GPS_Parse(GXGGA_Data,Gps_original_data);
		break;
		case 2:
			GPS_Location.GPS_Parse(GXGLL_Data,Gps_original_data);
		break;
		case 3:
			GPS_Location.GPS_Parse(GXRMC_Data,Gps_original_data);
		break;
		default:break;
	}
	
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
*/
void GPS::GPS_Unit_transform(void)
{
	#define EARTH_RADIUS 6371393
	#define PI 3.141592653
	#define LAT0 0
	#define LON0 0
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
	
	POS_X = (Lon_Deg*PI/180)*EARTH_RADIUS;		//经度 单位米	GGA
	POS_Y = (Lat_Deg*PI/180)*EARTH_RADIUS;		//纬度 单位米	GGA
	POS_Z = GXGGA_Data.AltSea; 								//相对海平面高度 单位米	GGA
	
	SatNum = GXGGA_Data.SatNum;
	state = GXGGA_Data.state;
}

