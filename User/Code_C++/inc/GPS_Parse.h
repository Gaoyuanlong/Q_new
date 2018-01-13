#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
#include "stm32f4xx.h"
#include "Common.h"
#define GPS_DATA_MAX_SIZE 250 //GPS֡����ַ���


class GXGGA
{
	public:
		GXGGA(){};
	
		float UTC;          //UTCʱ��
		float LAT;          //γ��
		char  NorS;         //�ϱ�
		float LON;          //����
		char WorE;          //����
		u8 state;           //��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
		u8 SatNum;          //��������
		float HDOP;         //ˮƽ����˥������
		float AltSea;       //��ƽ��߶�
		float AltEarth;     //����ں�ƽ��ĵ���߶�
		u16 GPSBaseTime;    //��һ���յ�GPS���վ���ݿ�ʼ�ļ�ʱ����λ��
		u16 GPSBase;        //���վ���
			
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


