#include "Attitude.h"
#define Kp 1.5f
#define Ki 0.0f
#define Z_FILTER 0.002f //������һ�׻����˲�ϵ��

Quaternion Q;//,Qz;
u8 IsCalulate = True;
struct Vector Angle;
struct Vector Rate;

/*
	����ϵ��ϵ˵����
	��������ϵ����MPU6050оƬ������ϵһ��  -> PWM����ӿڷ����ǻ�ͷ����
	            ˳�������᷽��ȥ��˳ʱ�뷽��Ϊ������
	            Pitch �� ������ϵY���˶�		Vector Angle.y
	            Roll  �� ������ϵX���˶�		Vector Angle.x
							Yaw   �� ������ϵZ���˶�   Vector Angle.z
	�ο�����ϵ������������ϵ  

*/


void Updata_Eular(void);

struct Attitude_ Attitude = 
{
	&Rate,
  &Angle,
	Updata_Eular,
};
#if 1
void Updata_Quaternion(Vector GYR,Vector ACC,Vector MAG,double DltaT)
{
	float Norm;
	double HalfT = DltaT / 2.0f;
	double vx,vy,vz;
	double ex,ey,ez;
	double gx,gy,gz;
	double ax,ay,az;
	static double exInt = 0, eyInt = 0, ezInt = 0;//������̬�������Ļ���
	
	Vector Mxyz;			//�����Ʋ���ֵ
	Vector MAG_Earth;
	double MAG_Z_angle;
	
	/***************************************************
	����gx��gy��gz�ֱ��Ӧ������Ľ��ٶȣ���λ�ǻ���/��
������ax��ay��az�ֱ��Ӧ������ļ��ٶ�ԭʼ����
	***************************************************/
	
	ax = ACC.x;
	ay = ACC.y;
	az = ACC.z;

	gx = Radians(GYR.x * MPU6050_GYR_2000DPS);
	gy = Radians(GYR.y * MPU6050_GYR_2000DPS);
	gz = Radians(GYR.z * MPU6050_GYR_2000DPS);	
	
	Attitude.Rate->x = gx;
	Attitude.Rate->y = gy;
	Attitude.Rate->z = gz;
	
	
	//�����ٶȵ�ԭʼ���ݣ���һ�����õ���λ���ٶ�
	arm_sqrt_f32(ax * ax + ay * ay + az * az,&Norm);
	if(Norm == 0) return;
	ax = ax / Norm; 
	ay = ay / Norm;
	az = az / Norm;
	
	/**************************************************
	����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������
	ת����������ϵ��������������Ԫ�ء����������vx��vy��vz��
	��ʵ���ǵ�ǰ�Ļ����������ϵ�ϣ����������������λ������
	(�ñ�ʾ������̬����Ԫ�����л���)
	***************************************************/
	vx = 2.0f * (Q.q2 * Q.q4 - Q.q1 * Q.q3);
	vy = 2.0f * (Q.q1 * Q.q2 + Q.q3 * Q.q4);
	vz = 1.0f - 2.0f * ( Q.q2 * Q.q2 + Q.q3 * Q.q3);//Q.w * Q.w + Q.z * Q.z;
	
	/***************************************************
	���������������������(Ҳ����������)����ʾ��	ex��
	ey��ez�����������������Ĳ���������������Ծ���λ�ڻ���
	����ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�
	С�����ݻ����������ȣ����������������ݡ����������ǶԻ�
	��ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ��
	������
	***************************************************/
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	
	/***************************************************
	�ò���������PI����������ƫ��ͨ������Kp��Ki������������
	�Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶ�
	***************************************************/
	if(Ki > 0)
	{
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}
	else
	{
		gx = gx + Kp * ex;
		gy = gy + Kp * ey;
		gz = gz + Kp * ez;   
	}
	
	//��Ԫ��΢�ַ��� 
	Q.q1 += (-Q.q2 * gx - Q.q3 * gy - Q.q4 * gz) * HalfT;
	Q.q2 += ( Q.q1 * gx + Q.q3 * gz - Q.q4 * gy) * HalfT;
	Q.q3 += ( Q.q1 * gy - Q.q2 * gz + Q.q4 * gx) * HalfT;
	Q.q4 += ( Q.q1 * gz + Q.q2 * gy - Q.q3 * gx) * HalfT;

	//��Ԫ����λ��
	arm_sqrt_f32(Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3 + Q.q4 * Q.q4,&Norm);
	
	if(Norm == 0) return;
	
	Q.q1 = Q.q1 / Norm;
	Q.q2 = Q.q2 / Norm;
	Q.q3 = Q.q3 / Norm;
	Q.q4 = Q.q4 / Norm;
	
	Angle.x = Degrees(atan2f(2.0f*(Q.q1*Q.q2 + Q.q3*Q.q4),1 - 2.0f*(Q.q2*Q.q2 + Q.q3*Q.q3)));
	Angle.y = Degrees(Safe_Asin(2.0f*(Q.q1*Q.q3 - Q.q2*Q.q4)));
	//Angle.z = Degrees(atan2f(2.0f*(Q.q2*Q.q3 + Q.q1*Q.q4),1 - 2.0f*(Q.q3*Q.q3 + Q.q4*Q.q4)));	
	
	
	
	Mxyz = MAG;
	arm_sqrt_f32(Mxyz.x*Mxyz.x + Mxyz.y*Mxyz.y + Mxyz.z*Mxyz.z,&Norm);       //mag���ݹ�һ��
		if(Norm == 0) return;//����ֵ�˳��������������
	Mxyz.x = Mxyz.x / Norm;
	Mxyz.y = Mxyz.y / Norm;
	Mxyz.z = Mxyz.z / Norm;	
	
	Angle.z += Degrees(gz*DltaT);																//�����ǻ���Z�Ƕ�
	if(Angle.z > 180)
			Angle.z = -180;
	if(Angle.z < -180)
			Angle.z = 180;	
	
	MAG_Earth = Math.Body_To_Earth(MAG,Angle.y ,Angle.x);				//����������ת�� ����->����
	MAG_Z_angle  = Degrees(atan2f(MAG_Earth.x,MAG_Earth.y));		//�����Ƽ���Ƕ�
	if(HMC5883.IsSensorError == False)													//������������������һ�׻����˲�
	{ 
		if(MAG_Z_angle - Angle.z > 180 )
			Angle.z = (1- Z_FILTER) * Angle.z+ Z_FILTER * (MAG_Z_angle - 360);
		else if(MAG_Z_angle - Angle.z < 180 )
			Angle.z = (1- Z_FILTER) * Angle.z + Z_FILTER * MAG_Z_angle;
	}
	if(Angle.z > 180)
			Angle.z = -180;
	if(Angle.z < -180)
			Angle.z = 180;	
	 
	User_Data.Data1 = MAG_Earth.x*100;  
	User_Data.Data2 = MAG_Earth.y*100;  
	User_Data.Data3 = MAG_Earth.z*100; 
	User_Data.Data4 = MAG_Z_angle;
	User_Data.Data5 = Angle.z;
//	Angle.y = Degrees(atan2f(2.0f*(Q.q1*Q.q3 - Q.q2*Q.q4),1 - 2.0f*(Q.q3*Q.q3 + Q.q4*Q.q4)));
//	Angle.z = Degrees(Safe_Asin(2.0f*(Q.q2*Q.q3 + Q.q1*Q.q4)));
//	Angle.x = Degrees(atan2f(2.0f*(Q.q1*Q.q2 - Q.q3*Q.q4),1 - 2.0f*(Q.q2*Q.q2 + Q.q4*Q.q4)));	
	
//	Angle.z = 0;
//	//˫ŷ����ȫ��̬����
//	if(abs(Angle.y) <= 45)
//	{

//	}
//	else
//	{
//		Angle.y = Degrees(atan2f(2.0f*(Q.q2*Q.q3 - Q.q1*Q.q4),1 - 2.0f*(Q.q3*Q.q3 + Q.q4*Q.q4)));
//		Angle.x = Degrees(atan2f(2.0f*(Q.q3*Q.q4 - Q.q1*Q.q2),1 - 2.0f*(Q.q2*Q.q2 + Q.q3*Q.q3)));
//		Angle.z = Degrees(Safe_Asin(2.0f*(Q.q1*Q.q3 + Q.q2*Q.q4)));
//	}
	
}

#endif

#if 0
void Updata_Quaternion(Vector GYR,Vector ACC,Vector MAG,double DltaT)
{
	float Norm;
	double HalfT = DltaT / 2.0f;
	double vx,vy,vz;			//���ٶ�����ֵ
	double wx,wy,wz;		//����������ֵ
	double ex,ey,ez;			//���
	double gx,gy,gz;			//�����ǲ���ֵ
	double ax,ay,az;			//���ٶȼƲ���ֵ
	double mx,my,mz;			//�����Ʋ���ֵ
	double hx, hy, hz, bx, bz; //������ˮƽֵ 
	static double exInt = 0, eyInt = 0, ezInt = 0;//������̬�������Ļ���
//--------------��ʼȡֵ����--------------------------------------------------------------------------------------------//	
	// �Ȱ���Щ�õõ���ֵ���
	double q1q1 = Q.q1*Q.q1;
	double q1q2 = Q.q1*Q.q2;
	double q1q3 = Q.q1*Q.q3;
	double q1q4 = Q.q1*Q.q4;
	double q2q2 = Q.q2*Q.q2;
	double q2q3 = Q.q2*Q.q3;
	double q2q4 = Q.q2*Q.q4;
	double q3q3 = Q.q3*Q.q3;
	double q3q4 = Q.q3*Q.q4;
	double q4q4 = Q.q4*Q.q4;

	/***************************************************
	����gx��gy��gz�ֱ��Ӧ������Ľ��ٶȣ���λ�ǻ���/��
������ax��ay��az�ֱ��Ӧ������ļ��ٶ�ԭʼ����
������mx��my��mz�ֱ��Ӧ������Ĵ�����ԭʼ����
	***************************************************/
	gx = Radians(GYR.x * MPU6050_GYR_2000DPS);
	gy = Radians(GYR.y * MPU6050_GYR_2000DPS);
	gz = Radians(GYR.z * MPU6050_GYR_2000DPS);	
	Attitude.Rate->x = gx;
	Attitude.Rate->y = gy;
	Attitude.Rate->z = gz;	
	
	ax = ACC.x;
	ay = ACC.y;
	az = ACC.z;

	mx = MAG.x;
	my = MAG.y;
	mz = MAG.z;
	
	//��һ�����õ���λ��
	arm_sqrt_f32(ax * ax + ay * ay + az * az,&Norm);  //acc���ݹ�һ��
		if(Norm == 0) return;//����ֵ�˳��������������
	ax = ax / Norm; 
	ay = ay / Norm;
	az = az / Norm;
	
	arm_sqrt_f32(mx*mx + my*my + mz*mz,&Norm);       //mag���ݹ�һ��
		if(Norm == 0) return;//����ֵ�˳��������������
	mx = mx / Norm;
	my = my / Norm;
	mz = mz / Norm;

	User_Data.Data1 = mx*100;  
	User_Data.Data2 = my*100;  
	User_Data.Data3 = mz*100; 
//---------------------------------------------------------------------------------------------------------------------//	
//--------------���ٶȼƲ���--------------------------------------------------------------------------------------------//	
	/**************************************************
	����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
	�������Ҿ����ŷ���ǵĶ��壬��������ϵ������������
	ת����������ϵ��������������Ԫ�ء����������vx��vy��vz��
	��ʵ���ǵ�ǰ�Ļ����������ϵ�ϣ����������������λ������
	(�ñ�ʾ������̬����Ԫ�����л���)
	***************************************************/
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = 1.0f - 2.0f * (q2q2 + q3q3);//Q.w * Q.w + Q.z * Q.z;
//---------------------------------------------------------------------------------------------------------------------//	
//------------�����Ʋ���-----------------------------------------------------------------------------------------------//	
	hx = 2 * mx * (0.5 - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) 				+ 2 * mz * (q2q4 + q1q3);  
	hy = 2 * mx * (q2q3 + q1q4) 			+ 2 * my * (0.5 - q2q2 - q4q4) 	+ 2 * mz * (q3q4 - q1q2);  
	hz = 2 * mx * (q2q4 - q1q3) 			+ 2 * my * (q3q4 + q1q2) 				+ 2 * mz * (0.5 - q2q2 -q3q3);          
	bx = sqrt((hx*hx) + (hy*hy));  
	bz = hz;

	User_Data.Data4 = bx*100;  
	User_Data.Data5 = bz*100; 
	
	wx = -2 * bx * (0.5 - q3q3 - q4q4) - 2 * bz * (q2q4 - q1q3);  
	wy = -2 * bx * (q2q3 - q1q4) 			- 2 * bz * (q1q2 + q3q4);  
	wz = -2 * bx * (q1q3 + q2q4) 			- 2 * bz * (0.5 - q2q2 - q3q3); 
	
	User_Data.Data6 = wx*100;  
	User_Data.Data7 = wy*100;  
	User_Data.Data8 = wz*100;
//---------------------------------------------------------------------------------------------------------------------//	
//------------�ںϲ���--------------------------------------------------------------------------------------------------//	
	/***************************************************
	���������������������(Ҳ����������)����ʾ��	ex��
	ey��ez�����������������Ĳ���������������Ծ���λ�ڻ���
	����ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ�����Ҳ���Ĵ�
	С�����ݻ����������ȣ����������������ݡ����������ǶԻ�
	��ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ��
	������
	***************************************************/
	ex = (ay*vz - az*vy) ;//+ (mz*wy - my*wz);  
  ey = (az*vx - ax*vz) ;//+ (mx*wz - mz*wx);  
  ez = (ax*vy - ay*vx) ;//+ (my*wx - mx*wy);
	
	/***************************************************
	�ò���������PI����������ƫ��ͨ������Kp��Ki������������
	�Կ��Ƽ��ٶȼ����������ǻ�����̬���ٶ�
	***************************************************/
	if(Ki > 0)
	{
		exInt = exInt + ex * Ki;
		eyInt = eyInt + ey * Ki;
		ezInt = ezInt + ez * Ki;
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}
	else
	{
		gx = gx + Kp * ex;
		gy = gy + Kp * ey;
		gz = gz + Kp * ez;   
	}
	
	//��Ԫ��΢�ַ��� 
	Q.q1 += (-Q.q2 * gx - Q.q3 * gy - Q.q4 * gz) * HalfT;
	Q.q2 += ( Q.q1 * gx + Q.q3 * gz - Q.q4 * gy) * HalfT;
	Q.q3 += ( Q.q1 * gy - Q.q2 * gz + Q.q4 * gx) * HalfT;
	Q.q4 += ( Q.q1 * gz + Q.q2 * gy - Q.q3 * gx) * HalfT;
//---------------------------------------------------------------------------------------------------------------------//	
//--------------��Ԫ��תŷ���ǲ���--------------------------------------------------------------------------------------//	
	//��Ԫ����λ��
	arm_sqrt_f32(Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3 + Q.q4 * Q.q4,&Norm);
	
	if(Norm == 0) return;
	
	Q.q1 = Q.q1 / Norm;
	Q.q2 = Q.q2 / Norm;
	Q.q3 = Q.q3 / Norm;
	Q.q4 = Q.q4 / Norm;
	
	Angle.x = Degrees(atan2f(2.0f*(Q.q1*Q.q2 + Q.q3*Q.q4),1 - 2.0f*(Q.q2*Q.q2 + Q.q3*Q.q3)));
	Angle.y = Degrees(Safe_Asin(2.0f*(Q.q1*Q.q3 - Q.q2*Q.q4)));
	//Angle.z = Degrees(atan2f(2.0f*(Q.q2*Q.q3 + Q.q1*Q.q4),1 - 2.0f*(Q.q3*Q.q3 + Q.q4*Q.q4)));
	
	//Angle.z += gz*DltaT;
}
#endif

/***************************************************
* ����ŷ����
* ÿ��2MS����һ����̬��
***************************************************/

void Updata_Eular(void)
{
	double DltaT;
	uint64_t Time_Now = 0;
	static uint64_t Time_Pre = 0;
	
	Time_Now = SystemTime.Now_US();
	DltaT = (Time_Now - Time_Pre) * (double)1e-6;
	Time_Pre = Time_Now;
	
	Updata_Quaternion(MPU6050.Data->GYR_ADC,MPU6050.Data->ACC_ADC,HMC5883.Data->MAG_ADC,DltaT);	

}





