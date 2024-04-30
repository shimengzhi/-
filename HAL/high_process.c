#include "mpu6050.h"
#include "attitude_process.h"
#include "high_process.h"
#include "sys.h"
#include "delay.h"


_st_Height Height;

 float pos_correction;
 float acc_correction;
 float vel_correction;
/****��ѹ�����׻����˲����������ο���Դ�ɿ�APM****/
//#define TIME_CONTANST_ZER       1.5f
const float TIME_CONTANST_ZER=3.0f;
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))//20															// XY????���䨤??�̨�y,3.0
#define K_POS_ZER               (3.0f / TIME_CONTANST_ZER)
float high_test;

void Strapdown_INS_High(float high)
{
			float dt;
			float Altitude_Estimate=0;
	    const uint8_t High_Delay_Cnt=2;//150ms
	    static float History_Z[High_Delay_Cnt+1]; 
			float temp;
	    float speed_Z;
			uint8_t Cnt;
			static float t_high;
			static float t_speed;
			static float last_accZ;
	    static uint16_t Save_Cnt=0;
			static float Current_accZ; //��ǰZ���ϵİ����ٶ�
			static uint16_t delay_cnt;
			static float high_offset;
	    {
			  static	float last_time;
				float now_time;
				now_time = micros();
				Height.sutgbl = micros(); 
				dt = now_time - last_time;
				dt /=1000;
//				dt = 0.003f;
				last_time = now_time;
			
			}
			if(delay_cnt<500)//�ո��ϵ�ʱ ����ѹ�Ƶ��油��
			{
				pos_correction = acc_correction = vel_correction = 0;
				high_offset = high;
				delay_cnt++;
				return;
			}
			high -= high_offset;

			
      Altitude_Estimate=high;//�߶ȹ۲���

			high_test = high;
      //�ɹ۲�������ѹ�ƣ��õ�״̬���
			
      temp= Altitude_Estimate- History_Z[High_Delay_Cnt];//��ѹ��(������)��SINS�������Ĳ��λcm
      //��·���ַ����������ߵ�
      acc_correction +=temp* K_ACC_ZER*dt ;//���ٶȽ�����
      vel_correction +=temp* K_VEL_ZER*dt ;//�ٶȽ�����
      pos_correction +=temp* K_POS_ZER*dt ;//λ�ý�����
      //���ٶȼƽ��������

      last_accZ=Current_accZ;//��һ�μ��ٶ���
      Current_accZ = Attitude.acc_yaw_sensor + acc_correction;
      //�ٶ�������������£����ڸ���λ��,���ڲ���h=0.005,��Խϳ���
      //������ö����������������΢�ַ��̣��������ø��߽׶Σ���Ϊ���ٶ��źŷ�ƽ��
			
      speed_Z =+(last_accZ+Current_accZ)*dt/2.0f;
      //ԭʼλ�ø���
      t_high += (Height.Speed+0.5f*speed_Z)*dt;
      //λ�ý��������
			Height.High = t_high + pos_correction;
      //��ֱԭʼ�ٶȸ���
      t_speed +=speed_Z;
      //��ֱ�ٶȽ��������
      Height.Speed = t_speed + vel_correction;

			//---------------------------------------------------------------
			
      Save_Cnt++;//���ݴ洢����
			
			
      if(Save_Cnt>=1)//5ms
      {
        for(Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//5ms����һ��
        {
					History_Z[Cnt]=History_Z[Cnt-1];
        }
        History_Z[0]=Height.High; //
        Save_Cnt=0;
      }
}







