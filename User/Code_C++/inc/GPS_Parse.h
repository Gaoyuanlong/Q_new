#ifndef __GPS_PARSE_H__
#define __GPS_PARSE_H__
#include "stm32f4xx.h"
#include "Common.h"
#define GPS_DATA_MAX_SIZE 100 //GPS֡����ַ���


class GXGGA
{
	public:
		GXGGA(){
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
		};
	
		double UTC;          //UTCʱ��
		double LAT;          //γ��
		char  NorS;         //�ϱ�
		double LON;          //����
		char WorE;          //����
		u8 state;           //��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
		u8 SatNum;          //��������
		double HDOP;         //ˮƽ����˥������
		double AltSea;       //��ƽ��߶�
		double AltEarth;     //����ں�ƽ��ĵ���߶�
		u16 GPSBaseTime;    //��һ���յ�GPS���վ���ݿ�ʼ�ļ�ʱ����λ��
		u16 GPSBase;        //���վ���
		
		void GXGGA_Data_Clear(void);
		private:
};
class GXRMC
{
		public:
		GXRMC(){
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
		};
	
		double UTC;         //UTCʱ��
		u8 state;           //��λ״̬ 0��δ��λ 1���޲�֣�SPSģʽ 2������֣�SPSģʽ 3��PPSģʽ
		double LAT;         //γ��
		char  NorS;         //�ϱ�
		double LON;         //����
		char WorE;          //����
		double SOG;  				//�������ʣ�000.0~999.9�ڣ�ǰ���0Ҳ�������䣩
		double COG;					//���溽��000.0~359.9�ȣ����汱Ϊ�ο���׼��ǰ���0Ҳ�������䣩
		double Date;        //���� ddmmyy�������꣩��ʽ
		double Mag_Var; 		//��ƫ�� ��000.0~180.0�ȣ�ǰ���0Ҳ�������䣩
		char Mag_EorW;			//��ƫ�Ƿ��� E��������W������
		char Mode;  				//ģʽָʾ����NMEA0183 3.00�汾�����A=������λ��D=��֣�E=���㣬N=������Ч��

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


