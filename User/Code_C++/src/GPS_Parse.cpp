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

//-------GXGGA���ݽ���---------------------------------------//
void GXGGA::GXGGA_Data_Clear(void)
{
		 UTC = 0;          //UTCʱ��
		 LAT = 0;          //γ��
		 NorS = 0;         //�ϱ�
		 LON = 0;          //����
		 WorE = 0;          //����
		 state = 0;           //��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
		 SatNum = 0;          //��������
		 HDOP = 0;         //ˮƽ����˥������
		 AltSea = 0;       //��ƽ��߶�
		 AltEarth = 0;     //����ں�ƽ��ĵ���߶�
		 GPSBaseTime = 0;    //��һ���յ�GPS���վ���ݿ�ʼ�ļ�ʱ����λ��
		 GPSBase = 0;        //���վ���
}	
void GPS::GPS_Parse(GXGGA &GXGGA_Data ,const u8* BUF)
{
	#define ONE_DATA_MAX_SIZE 20
	const u8* Str_Add_temp = BUF;
	u16 i = 0;
	u8 j = 0;
	
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      	//1 UTC ʱ��
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.UTC = 0;
	else GXGGA_Data.UTC = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //2 γ��
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.LAT = 0;
	else GXGGA_Data.LAT = GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //3 N �� S
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.NorS = 0;
	else GXGGA_Data.NorS = Str_Add_temp[i];		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //4 ����
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.LON = 0;
	else GXGGA_Data.LON = GPS_Str2Float(&Str_Add_temp[i]);	
		 
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //5 E �� W
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.WorE = 0;
	else GXGGA_Data.WorE = Str_Add_temp[i];		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //6 ��λ����ָʾ��0=��λ��Ч��1=��λ��Ч
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.state = 0;
	else GXGGA_Data.state = Str_Add_temp[i] - '0';
				
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //7 ʹ�������������� 00 �� 12
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.SatNum = 0;
	else GXGGA_Data.SatNum = (u8)GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //8 ˮƽ��ȷ�ȣ�0.5 �� 99.9
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.HDOP = 0;
	else GXGGA_Data.HDOP = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //9 �����뺣ƽ��ĸ߶ȣ�-9999.9 �� 9999.9 ��
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.AltSea = 0;
	else GXGGA_Data.AltSea = GPS_Str2Float(&Str_Add_temp[i]);	
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      // ��λM ������
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //10 ���ˮ׼��߶ȣ�-9999.9 �� 9999.9 ��
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.AltEarth = 0;
	else GXGGA_Data.AltEarth = GPS_Str2Float(&Str_Add_temp[i]);
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      // ��λM ������
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //11 ��� GPS ��������
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.GPSBaseTime = 0;
	else GXGGA_Data.GPSBaseTime = (u16)GPS_Str2Float(&Str_Add_temp[i]);

	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 ��ֲο���վ���
	if(Str_Add_temp[i] ==  ',') GXGGA_Data.GPSBase = 0;
	else GXGGA_Data.GPSBase = (u16)GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  '*'){if(j++ > ONE_DATA_MAX_SIZE)return;}      //13 ��������־��

}
//-------GXRMC ���ݽ���---------------------------------------//
void GXRMC::GXRMC_Data_Clear(void)
{
		 UTC = 0;         //UTCʱ��
		 state = 0;           //��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
		 LAT = 0;         //γ��
		 NorS = 0;         //�ϱ�
		 LON = 0;         //����
		 WorE = 0;          //����
		 SOG = 0;  				//�������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
		 COG = 0;					//���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
		 Date = 0;        //���� ddmmyy�������꣩��ʽ
		 Mag_Var = 0; 		//��ƫ�� ��000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
		 Mag_EorW = 0;			//��ƫ�Ƿ��� E��������W������
		 Mode = 0;  				//ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��
}
void GPS::GPS_Parse(GXRMC &GXRMC_Data ,const u8* BUF)
{
	#define ONE_DATA_MAX_SIZE 20
	const u8* Str_Add_temp = BUF;
	u16 i = 0;
	u8 j = 0;
	
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      	//1 UTC ʱ��
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.UTC = 0;
	else GXRMC_Data.UTC = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //2 ��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.state = 0;
	else GXRMC_Data.state = Str_Add_temp[i];	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //3 γ��
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.LAT = 0;
	else GXRMC_Data.LAT = GPS_Str2Float(&Str_Add_temp[i]);		
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //4  N �� S
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.NorS = 0;
	else GXRMC_Data.NorS = Str_Add_temp[i];	
		 
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //5 ����
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.LON = 0;
	else GXRMC_Data.LON = GPS_Str2Float(&Str_Add_temp[i]);				
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //6 E �� W
	if(Str_Add_temp[i] ==  ',')  GXRMC_Data.WorE = 0;
	else GXRMC_Data.WorE = Str_Add_temp[i];
				
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //7 �������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.SOG = 0;
	else GXRMC_Data.SOG = GPS_Str2Float(&Str_Add_temp[i]);
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //8 ���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.COG = 0;
	else GXRMC_Data.COG = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //9 ���� ddmmyy�������꣩��ʽ
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Date = 0;
	else GXRMC_Data.Date = GPS_Str2Float(&Str_Add_temp[i]);	
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //10 ��ƫ�� ��000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mag_Var = 0;
	else GXRMC_Data.Mag_Var = GPS_Str2Float(&Str_Add_temp[i]);	
		
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 ��ƫ�Ƿ��� E��������W������
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mag_EorW = 0;
	else GXRMC_Data.Mag_EorW = Str_Add_temp[i];
	
	j = 0;
	while(Str_Add_temp[i++] !=  ','){if(j++ > ONE_DATA_MAX_SIZE)return;}      //12 ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��
	if(Str_Add_temp[i] ==  ',') GXRMC_Data.Mode = 0;
	else GXRMC_Data.Mode = Str_Add_temp[i];
		
	j = 0;
	while(Str_Add_temp[i++] !=  '*'){if(j++ > ONE_DATA_MAX_SIZE)return;}      //13 ��������־��
}
//-------GXGLL ���ݽ���---------------------------------------//
void GPS::GPS_Parse(GXGLL &GXGLL_Data ,const u8* BUF)
{

}
//-------��֡ͷ---------------------------------------//
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
		���һ��A�ľ� γ��Ϊ(LonA, LatA)���ڶ���B�ľ�γ��Ϊ(LonB, LatB)��
		����0�Ⱦ��ߵĻ�׼������ȡ���ȵ���ֵ(Longitude)������ȡ���ȸ�ֵ(-Longitude)��
		��γȡ90-γ��ֵ(90- Latitude)����γȡ90+γ��ֵ(90+Latitude)��
		�򾭹����������������㱻��Ϊ(MLonA, MLatA)��(MLonB, MLatB)��
		��ô���������Ƶ������Եõ����������������¹�ʽ��
		C = sin(MLatA)*sin(MLatB)*cos(MLonA-MLonB) + cos(MLatA)*cos(MLatB)
		Distance = R*Arccos(C)*Pi/180

		�ȷ�ת����
		���ȷֵ�λ����ת��Ϊ�ȵ�λ���� 
		��=��+��/60
		���磺
		���� = 116��20.12��
		γ�� = 39��12.34�� 
		���� = 116 + 20.12 / 60 = 116.33533�� 
		γ�� = 39 + 12.34 / 60 = 39.20567��

		������ʽ
		l = n��Բ�Ľǣ��� �У�Բ���ʣ��� r���뾶��/180=��(Բ�Ľǻ�����)�� r���뾶��
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
	
	POS_X = (Lon_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectX;		//���� ��λcm	GGA
	POS_Y = (Lat_Deg*PI/180)*EARTH_RADIUS * 100 - Home_OffectY;		//γ�� ��λcm	GGA
	POS_Z = GXGGA_Data.AltSea * 100 - Home_OffectZ; 								//��Ժ�ƽ��߶� ��λcm	GGA
	//λ����Ҫ�����˲� ����
	float Speed_Temp = GXRMC_Data.SOG * 51.4; 			//���� ��λcm/s   ��תcm/s
	float Angle_Temp = GXRMC_Data.COG * DEG_TO_RAD;	//����� 0-360 		����ת��
	Speed.x = Speed_Temp * arm_sin_f32(Angle_Temp);	// ��λcm/s
	Speed.y = Speed_Temp * arm_cos_f32(Angle_Temp);	// ��λcm/s
	//�ٶ���Ҫ�����˲�	����
	SatNum = GXGGA_Data.SatNum;
	state = GXGGA_Data.state;
	
	//λ��ƫ��ȡֵ  ��ʱʹ�� ������Ҫ�޸�
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




