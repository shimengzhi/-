#include "WIFI_UFO.h"
#include "Remote.h"
#include "ANO_DT.h"
#include "ALL_DATA.h"
#include "myMath.h"


////////////////////////////////////////////////////
//������WIFI_UFOͨ��Э�����
////////////////////////////////////////////////////
static uint8_t wifi_RxBuffer[8];
uint16_t WIFI_UFO_Err;
uint8_t WIFI_SSI,WIFI_SSI_CNT;//WIFI�ź�

static uint8_t wifi_PUI=0;

//WIFI_UFO���ݽ��պ���
void WIFI_UFO_Data_Receive_Prepare(uint8_t data)
{
	static uint8_t _data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0x66)
	{
		state=1;
		wifi_RxBuffer[0]=data;
	}
	else if(state==1)
	{
		wifi_RxBuffer[_data_cnt++]=data;
	}
	
	if(_data_cnt==7)
	{
		state = 0;
		_data_cnt = 0;
		if(data == 0x99)WIFI_UFO_Err=0;
	}
}
///////////////////////////////////////////////

static void Key_Function(u8 key)   //wifiģ��İ�������
{
	static u8 temp;
	
	if(temp != Remote.AUX2)//��־λ�ж�
	{
		switch(temp)
		{
			case 1://һ�����  �ֻ��򿪶���ģʽ����Ч
//				if(fly_ready==0)
//				{
//					fly_ready = 1;
//					high_start = HIGH_START;
//				}
				break;
			case 2://һ������  �ֻ��򿪶���ģʽ����Ч
//				if(fly_ready==1)
//				{
//					auto_landing = 1;
//				}
				break;
			case 8://3D���� ��һ������8 �ڰ�����0
      wifi_PUI = 1;
				break;
			case 16: //��ͷģʽ ��һ������16 �ڰ�����0

				break;
			case 128://����ͣ��   ���ð�������һ�½�3��128
				
				break;
			default:	
			wifi_PUI = 0;
			break;
		}
		temp = Remote.AUX2;
	}
}


static u8 wifi_RxBuffer22[8];

//wifi����ͷģ��APP�������ݽ���

//WIFI_UFO���ݴ�����   66 80 80 00 80 00 80 99
void WIFI_UFO_Data_Receive_Anl(u8 *data_buf,u8 num)   
{
  flag.NS = 2;//ң��������Դ��1��ң������2��WiFiͼ��ģ�顣3������ģ�飩  
/////////////////////////////////////////////////////
	//ң������
	Remote.thr = ((float)*(data_buf+2)/256) * 1000 + 1000;
	LIMIT(Remote.thr,1000,2000);
	Remote.yaw  = ((float)*(data_buf+3)*3.90625) + 1000;
	LIMIT(Remote.yaw,1000,2000);
//  Remote.roll = (((float)*(data_buf+0)-88)*12.5) + 1000;//12.5
	Remote.roll = (((float)*(data_buf+0)/256)*1000) + 1000;//
 	LIMIT(Remote.roll,1000,2000);
//	Remote.pitch = (((float)*(data_buf+1)-88)*12.5)+ 1000;//12.5
		Remote.pitch = (((float)*(data_buf+1)/256)*1000)+ 1000;//  
	LIMIT(Remote.pitch,1000,2000);
	
	wifi_RxBuffer22[0] = ((float)*(data_buf+0));
	
	wifi_RxBuffer22[1] = ((float)*(data_buf+1));
	
	wifi_RxBuffer22[2] = ((float)*(data_buf+2));

	wifi_RxBuffer22[3]  = ((float)*(data_buf+3));
	
  wifi_RxBuffer22[4] = ((float)*(data_buf+4));

	wifi_RxBuffer22[5] = ((float)*(data_buf+5));

	wifi_RxBuffer22[6] = ((float)*(data_buf+6));
	
	wifi_RxBuffer22[7] = ((float)*(data_buf+7));
	
	//////////////////////////////////////////////////////
	//��־λ
	Remote.AUX2 = *(data_buf+4);
//	LIMIT(Remote.AUX2,1000,2000);
	Key_Function(Remote.AUX2);
				
}

//WIFI_UFO���Ӻ���   
u8 WIFI_UFO_Connect(void)
{
	static u8 Connect_flag;
	static u8 count10=0;
	WIFI_UFO_Err ++;
	if(WIFI_UFO_Err==1)   //�ڽ��ճ���������
	{
		WIFI_UFO_Data_Receive_Anl(wifi_RxBuffer,8);
		WIFI_SSI_CNT++;
		Connect_flag = 1;
		count10++;
		if(count10>=2)
		{
			count10=0;		
			if(wifi_PUI==1)
			{	
//				printf("Remote.thr: %d  Remote.yaw: %d  Remote.roll: %d   Remote.pitch: %d  \r\n",Remote.thr,Remote.yaw,Remote.roll,Remote.pitch);  //����1���
			}
//			printf("rxbuf0: %d  rxbuf1: %d  rxbuf2: %d  rxbuf3: %d   rxbuf4: %d  rxbuf5: %d  rxbuf6: %d   rxbuf7: %d   \r\n",wifi_RxBuffer22[0],wifi_RxBuffer22[1],wifi_RxBuffer22[2],wifi_RxBuffer22[3],wifi_RxBuffer22[4],wifi_RxBuffer22[5],wifi_RxBuffer22[6],wifi_RxBuffer22[7]);  //����1���
		}
	}
	if(WIFI_UFO_Err>=30)
	{
		WIFI_UFO_Err = 1;
		Connect_flag = 0;
		flag.NS = 0;
//		printf("wifi©�� \r\n");  //����1���	
	}
	return Connect_flag;
}
