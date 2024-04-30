//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 799870988
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2020.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include <stdlib.h>
#include <string.h>
#include "ALL_DATA.h"
#include "ANO_DT.h"
#include "USART3.h"
#include "imu.h"
#include "myMath.h"
#include "ANO_Data_Transfer.h"
/******************************************************************/
//--------------------------
static uint8_t RatePID[18];
static uint8_t AnglePID[18];
static uint8_t HighPID[18];
static uint8_t HighPID4[18];
_flag_st flag;
static struct{
	uint8_t PID1 :1; //���ܵ���λ��PID��1
	uint8_t PID2 :1; //���ܵ���λ��PID��2
	uint8_t PID3 :1; //���ܵ���λ��PID��3
	uint8_t PID4 :1; //���ܵ���λ��PID��4
	uint8_t PID5 :1; //���ܵ���λ��PID��5
	uint8_t PID6 :1; //���ܵ���λ��PID��6	
	uint8_t CMD2_READ_PID:1; //���ܵ���λ����ȡPID������
}ANTO_Recived_flag;
 

/////////////////////////////////////////////////////////////////////////////////////


extern u16 save_pid_en;


/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/	
void ANO_Recive(int8_t *pt)                   //���յ���λ��������
{
	switch(pt[2])
	{
		case ANTO_RATE_PID:
			ANTO_Recived_flag.PID1 = 1;             //���յ���λ��������PID����
			memcpy(RatePID,&pt[4],18);              //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�
			break;
		case ANTO_ANGLE_PID:                      //�����PID�Ǹ��ǶȻ��õ�
			memcpy(AnglePID,&pt[4],18);
			ANTO_Recived_flag.PID2 = 1;
			break;
		case ANTO_HEIGHT_PID:                     //�����PID�Ǹ��߶Ȼ��õ�
			memcpy(HighPID,&pt[4],18);
			ANTO_Recived_flag.PID3 = 1;
			break;
		case ANTO_PID4:
			memcpy(HighPID4,&pt[4],18);
			ANTO_Recived_flag.PID4 = 1;
			break;
		case ANTO_PID5:   
			break;
		case ANTO_PID6:
			break;
		case 0x01:                                //��λ��������CMD1 ��������У׼

			break;
		case 0x02:                                //��λ��������CMD2 ���������ȡPID��
			{
			   enum                                  //��λ����ɿ�����
				{
					READ_PID = 0X01,                    //��ȡ�ɿص�PID����
					READ_MODE = 0x02,                   //��ȡ����ģʽ
					READ_ROUTE = 0x21,                  //��ȡ������Ϣ
					READ_VERSION = 0XA0,                //��ȡ�ɿذ汾
					RETURN_DEFAULT_PID = 0xA1           //�ָ�Ĭ��PID
				 }CMD2;

				switch(*(uint8_t*)&pt[4])             //�ж���λ������CMD������
				{
					case READ_PID:                      //��λ�������ȡ�ɿ�PID����
						ANTO_Recived_flag.CMD2_READ_PID = 1;
						break;
					case READ_MODE: 
						break;
					case READ_ROUTE: 
						break;					
					case READ_VERSION:  
						break;
					case RETURN_DEFAULT_PID:  
						break;					
					default: 
						break;					
				}
			
			}
			break;
		case ANTO_RCDATA: //Immediately deal with 
			break;
		default:
			break;			
	}
	return;
}
/***********************************************************************
 * 
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
static void ANTO_Send(const enum ANTO_SEND FUNCTION) //�������ݵ���λ��
{
	uint8_t i;
	uint8_t len=2;
	int16_t Anto[12];
	int8_t *pt = (int8_t*)(Anto);
	PidObject *pidX=0;
	PidObject *pidY=0;
	PidObject *pidZ=0;
  long _temp;

	
	switch(FUNCTION)
	{
		case ANTO_RATE_PID:      //PID1
				 pidX = &pidRateX;
				 pidY = &pidRateY;
				 pidZ = &pidRateZ;
         goto send_pid;		
		case ANTO_ANGLE_PID:      //PID2
				 pidX = &pidRoll;
				 pidY = &pidPitch;
				 pidZ = &pidYaw;
				 goto send_pid;				
		case ANTO_HEIGHT_PID:     //PID3
				 pidX = &pidHeightRate;
				 pidY = &pidHeightHigh;
				 pidZ = &Flow_SpeedPid_x;
				 goto send_pid;							
		case ANTO_PID4:	 				 //PID4
				 pidX = &Flow_PosPid_x;
				 pidY = &Flow_SpeedPid_y;
				 pidZ = &Flow_PosPid_y;
				 goto send_pid;	
		case ANTO_PID5:	         //PID5
    case ANTO_PID6:
send_pid:
			if(pidX!=NULL)
			{
				Anto[2] = (int16_t)(pidX->kp *1000);
				Anto[3] = (int16_t)(pidX->ki *1000);
				Anto[4] = (int16_t)(pidX->kd *1000);
			}
			if(pidY!=NULL)
			{
				Anto[5] = (int16_t)(pidY->kp *1000);
				Anto[6] = (int16_t)(pidY->ki *1000);
				Anto[7] = (int16_t)(pidY->kd *1000);
			}
			if(pidZ!=NULL)
			{
				Anto[8] = (int16_t)(pidZ->kp *1000);
				Anto[9] = (int16_t)(pidZ->ki *1000);
				Anto[10] = (int16_t)(pidZ->kd *1000);
			}
			len = 18;
			break;
		case ANTO_MOTOR:    //send motor

				len = 8;
			break;	
		case ANTO_RCDATA:            //����ң������ͨ������
		   	Anto[2] = Remote.thr;
				Anto[3] = Remote.yaw;
				Anto[4] = Remote.roll;
				Anto[5] = Remote.pitch;
				Anto[6] = Remote.AUX1;
				Anto[7] = Remote.AUX2;
				Anto[8] = Remote.AUX3;
				Anto[9] = Remote.AUX4;
				Anto[10] =Remote.AUX5; 
				Anto[11] =Remote.AUX6; 
				len = 20;

			break;
		case ANTO_MPU_MAGIC:     //����MPU6050�ʹ����Ƶ�����
			memcpy(&Anto[2],(int8_t*)&MPU6050,sizeof(_st_Mpu));
			memcpy(&Anto[8],(int8_t*)&AK8975,sizeof(_st_Mag));
			len = 18;
			break;
		case ANTO_SENSER2:	
				if(spl_flag==1)  //��ѹ��־λ �ж���ѹоƬ�Ƿ񺸽�
		    {
						_temp= (long)FlightData.High.ultra_baro_height;						//��ѹ��ģ��ĸ߶� 
					
					
				}
				else _temp=0;
				*(uint16_t*)&Anto[2] = *((uint16_t*)&_temp+1);
				*(uint16_t*)&Anto[3] = *((uint16_t*)&_temp);
				Anto[4] = (uint32_t)FlightData.High.ultra_height;   //������ģ��߶�
				//Anto[4] = 0;
				len = 6;
		    
	
			break;
		case ANTO_STATUS:     //send angle
			
		  	if(spl_flag==1)  //��ѹ��־λ �ж���ѹоƬ�Ƿ񺸽�
		    {
						 _temp= ((long)FlightData.High.bara_height);						//��ѹ��ģ��ĸ߶�
				}
				else _temp=0;
			
		
				Anto[2] = (int16_t)(-Angle.roll*100);
				Anto[3] = (int16_t)(Angle.pitch*100);
				Anto[4] = (int16_t)(-Angle.yaw*100);
				*(uint16_t*)&Anto[5] = *((uint16_t*)&_temp+1);
				*(uint16_t*)&Anto[6] = *((uint16_t*)&_temp);  				//�߶�	 ��16λ	
				switch(Command.FlightMode)
				{
						case LOCK:
							ALL_flag.slock_flag = 0;  //��λ����ģʽ
							break;
						case NORMOL:
								if(ALL_flag.unlock == 1)
								{
										
									ALL_flag.slock_flag = 0x0101; 
								}
								else{ALL_flag.slock_flag = 0x0100;}
							break;
						case HEIGHT:
								if(ALL_flag.unlock == 1)
								{
										ALL_flag.slock_flag = 0x0201;
								}
								else{ALL_flag.slock_flag = 0x0200;}
						  break;
						case Flow_POSITION:
								if(ALL_flag.unlock == 1)
								{
										ALL_flag.slock_flag= 0x0301;
								}
								else{ALL_flag.slock_flag= 0x0300;}
							break;
						default:
							ALL_flag.slock_flag = 0;  
							break;
				}	
				Anto[7] =ALL_flag.slock_flag ;	
				len = 12;
			break;
		case ANTO_POWER:

				break;
		default:
			break;			
	}
	
	Anto[0] = 0XAAAA;
	Anto[1] = len | FUNCTION<<8;
	pt[len+4] = (int8_t)(0xAA+0xAA);
	for(i=2;i<len+4;i+=2)    //a swap with b;
	{
		pt[i] ^= pt[i+1];
		pt[i+1] ^= pt[i];
		pt[i] ^= pt[i+1];
		pt[len+4] += pt[i] + pt[i+1];
	}
	
	USART3_SendByte(pt,len+5);
}
/***********************************************************************
 * polling  work.
 * @param[in] 
 * @param[out] 
 * @return     
 **********************************************************************/
void ANTO_polling(void) //��ѯɨ����λ���˿�
{
	volatile static uint8_t status = 0;
	switch(status)
	{
		case 0:
			
			status = 1;
			break;
		case 1:


			if(*(uint8_t*)&ANTO_Recived_flag != 0) //һ�����յ���λ�������ݣ�����ͣ�������ݵ���λ����ת��ȥ�ж���λ��Ҫ��ɿ���ʲô��
			{
				status = 2;
			}
			else
			{
					ANTO_Send(ANTO_MPU_MAGIC);
		//			delay_ms(1);
					ANTO_Send(ANTO_STATUS);
		//			delay_ms(1);
					ANTO_Send(ANTO_RCDATA);
		//			delay_ms(1);
					ANTO_Send(ANTO_SENSER2);
		//			delay_ms(1);
			}
		 	break;
		case 2:
			if(*(uint8_t*)&ANTO_Recived_flag == 0)//��λ���ķ����������ݶ��������ˣ��򷵻�ר�ĵķ������ݵ���λ��
			{
				status = 1;
			}
	
			if(ANTO_Recived_flag.CMD2_READ_PID) //�ж���λ���Ƿ����󷢷��ͷɿ�PID���ݵ���λ��
			{		
					ANTO_Send(ANTO_RATE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_ANGLE_PID);
					delay_ms(1);
					ANTO_Send(ANTO_HEIGHT_PID);
					delay_ms(1);
				  ANTO_Send(ANTO_PID4);
					delay_ms(1);
					ANTO_Recived_flag.CMD2_READ_PID = 0;
			}
			
			if(*(uint8_t*)&ANTO_Recived_flag & 0x3f) //���յ���λ��������PID����
			{
					PidObject *pidX=0;
					PidObject *pidY=0;
					PidObject *pidZ=0;
				  uint8_t *P;
				
					if(ANTO_Recived_flag.PID1)
					{
						 pidX = &pidRateX;
						 pidY = &pidRateY;
						 pidZ = &pidRateZ;
						 P = RatePID;
						 ANTO_Recived_flag.PID1 = 0;
					}
					else if(ANTO_Recived_flag.PID2)
					{
						 pidX = &pidRoll;
						 pidY = &pidPitch;
						 pidZ = &pidYaw;
						 P = AnglePID;	
						 ANTO_Recived_flag.PID2 = 0;                             
					}
					else if(ANTO_Recived_flag.PID3)
					{
						 pidX = &pidHeightRate;
						 pidY = &pidHeightHigh;
						 pidZ = &Flow_SpeedPid_x;
						 P = HighPID;	
						 ANTO_Recived_flag.PID3 = 0;     						
					}
					else if(ANTO_Recived_flag.PID4)
					{
						 pidX = &Flow_PosPid_x;
						 pidY = &Flow_SpeedPid_y;
						 pidZ = &Flow_PosPid_y;
						 P = HighPID4;	
						 ANTO_Recived_flag.PID4 = 0;     						
					}
					else
					{
						break;
					}
					{
							union {
								uint16_t _16;
								uint8_t _u8[2];
							}data;
							
							if(pidX!=NULL)
							{
								data._u8[1] = P[0]; 
								data._u8[0] = P[1];
								pidX->kp =  data._16 /1000.0f;
								data._u8[1] = P[2]; 
								data._u8[0] = P[3];
								pidX->ki =  data._16 /1000.0f;
								data._u8[1] = P[4]; 
								data._u8[0] = P[5];
								pidX->kd =  data._16 /1000.0f;								
							}
							if(pidY!=NULL)
							{
								data._u8[1] = P[6]; 
								data._u8[0] = P[7];
								pidY->kp =  data._16 /1000.0f;
								data._u8[1] = P[8]; 
								data._u8[0] = P[9];
								pidY->ki =  data._16 /1000.0f;
								data._u8[1] = P[10]; 
								data._u8[0] = P[11];
								pidY->kd =  data._16 /1000.0f;		
							}
							if(pidZ!=NULL)
							{
								data._u8[1] = P[12]; 
								data._u8[0] = P[13];
								pidZ->kp =  data._16 /1000.0f;
								data._u8[1] = P[14]; 
								data._u8[0] = P[15];
								pidZ->ki =  data._16 /1000.0f;
								data._u8[1] = P[16]; 
								data._u8[0] = P[17];
								pidZ->kd =  data._16 /1000.0f;		
							}				
					}				
			}
			break;
		default:
			break;
	}

}
////////////////////////////////////////////////////////////////////////
////////////////////////����3�жϽ��ճ���///////////////////////////////
////////////////////////////////////////////////////////////////////////
static int8_t CheckSend[7]={0xAA,0XAA,0xEF,2,0,0,0};
u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0;

void USART3_IRQHandler(void) //����3�жϽ��ճ���
{ 	
	if((USART3->SR & (1<<7))&&(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART3->DR = TxBuffer3[TxCounter3++]; //дDR����жϱ�־ 
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//�ر�TXE�ж�
		}
	}
	//�����ж�
	if(USART3->SR & (1<<5))//if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)    
	{
		u8 com_data = USART3->DR;
 	  ANO_DT_Data_Receive_Prepare(com_data);//��λ�����ݽ���
	}

}

//////////////////У���뷵����λ��////////////////////////////////////////////////////
static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
{
	  u8 sum = 0,i=0;
    data_to_send[0]=CheckSend[0]=0xAA; 
    data_to_send[1]=CheckSend[1]=0xAA;
    data_to_send[2]=CheckSend[2]=0xEF;
    data_to_send[3]=CheckSend[3]=2;
    data_to_send[4]=CheckSend[4]=head;
    data_to_send[5]=CheckSend[5]=check_sum;
    for(i=0;i<6;i++) 
        sum += CheckSend[i];
    data_to_send[6]=CheckSend[6]=sum;
	
    Send_Check = 1;  //ң�������·������ı�־λ
	
    USART3_SendByte(CheckSend,7);  //���ڷ���
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare������Э��Ԥ����������Э��ĸ�ʽ�����յ������ݽ���һ�θ�ʽ�Խ�������ʽ��ȷ�Ļ��ٽ������ݽ���
//��ֲʱ���˺���Ӧ���û���������ʹ�õ�ͨ�ŷ�ʽ���е��ã����紮��ÿ�յ�һ�ֽ����ݣ�����ô˺���һ��
//�˺������������ϸ�ʽ������֡�󣬻����е������ݽ�������
void ANO_DT_Data_Receive_Prepare(u8 data) 
{
    static u8 RxBuffer[50];
    static u8 _data_len = 0,_data_cnt = 0;
    static u8 state = 0;

    if(state==0&&data==0xAA)  //��ͷ
    {
        state=1;
        RxBuffer[0]=data;
    }
    else if(state==1&&data==0xAF) //��ͷ
    {
        state=2;
        RxBuffer[1]=data;
    }
    else if(state==2&&data<0XF1) //�����ֽ�
    {
        state=3;
        RxBuffer[2]=data;
    }
    else if(state==3&&data<50)  //����
    {
        state = 4;
        RxBuffer[3]=data;
        _data_len = data;  //��¼����
        _data_cnt = 0;
    }
    else if(state==4&&_data_len>0)  //��ʼһ���������ݽ���
    {
        _data_len--;
        RxBuffer[4+_data_cnt++]=data;
        if(_data_len==0)
            state = 5;
					
    }
    else if(state==5)
    {
        state = 0;
        RxBuffer[4+_data_cnt]=data;
        ANO_DT_Data_Receive_Anl(RxBuffer,_data_cnt+5);//���ݽ�������
    }
    else
        state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���Data_Receive_Prepare�Զ�����
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{

    u8 sum = 0,i=0;
    for(i=0;i<(num-1);i++)
        sum += *(data_buf+i);
    if(!(sum==*(data_buf+num-1)))       return;     //�ж�sum
    if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))     return;     //�ж�֡ͷ

    if(*(data_buf+2)==0X01)     //��λ�������İ�������У׼
    {
        if(*(data_buf+4)==0X01)   
            ;//mpu6050.Acc_CALIBRATE = 1;
        if(*(data_buf+4)==0X02)
            ;//mpu6050.Gyro_CALIBRATE = 1;
        if(*(data_buf+4)==0X03)
        {
            ;//mpu6050.Acc_CALIBRATE = 1;
            ;//mpu6050.Gyro_CALIBRATE = 1;
        }
    }
 
    else if(*(data_buf+2)==0X02)    //��ȡ�ɻ���Ϣ
    {

			if(*(data_buf+4)==0X01)
        {
					ANTO_Recived_flag.CMD2_READ_PID = 1;   //��ȡPID ��Ϣ
					f.send_pid = 1;
					cnt = 0;
        }
        if(*(data_buf+4)==0X02)
        {

        }
        if(*(data_buf+4)==0XA0)     //��ȡ�汾��Ϣ
        {
            f.send_version = 1;
        }
        if(*(data_buf+4)==0XA1)     //�ָ�Ĭ�ϲ���
        {
 //           Sort_PID_Flag=2;
					pid_param_Init();
        }
    }
		 else if(*(data_buf+2)==0X03)    //����ң��������
		 {	
			 
				flag.NS = 1;//??????(1:????2:WiFi?????3:????)
				Remote.thr = (vs16)(*(data_buf+4)<<8)|*(data_buf+5) ;
			  LIMIT(Remote.thr,1000,2000);
				Remote.yaw = (vs16)(*(data_buf+6)<<8)|*(data_buf+7) ;
				LIMIT(Remote.yaw,1000,2000);
				Remote.roll = (vs16)(*(data_buf+8)<<8)|*(data_buf+9) ;
			  LIMIT(Remote.roll,1000,2000);
				Remote.pitch = (vs16)(*(data_buf+10)<<8)|*(data_buf+11) ;
			  LIMIT(Remote.pitch,1000,2000);
				Remote.AUX1 = (vs16)(*(data_buf+12)<<8)|*(data_buf+13) ;
			  LIMIT(Remote.AUX1,1000,2000);
				Remote.AUX2 = (vs16)(*(data_buf+14)<<8)|*(data_buf+15) ;
			  LIMIT(Remote.AUX2,1000,2000);
				Remote.AUX3 = (vs16)(*(data_buf+16)<<8)|*(data_buf+17) ;
			  LIMIT(Remote.AUX3,1000,2000);
				Remote.AUX4 = (vs16)(*(data_buf+18)<<8)|*(data_buf+19) ;
			  LIMIT(Remote.AUX4,1000,2000);
				Remote.AUX5 = (vs16)(*(data_buf+20)<<8)|*(data_buf+21) ;
			  LIMIT(Remote.AUX5,1000,2000);
				Remote.AUX6 = (vs16)(*(data_buf+22)<<8)|*(data_buf+23) ;
				LIMIT(Remote.AUX6,1000,2000);
	//			key_function(RX_CH[AUX4]);
		 }
    else if(*(data_buf+2)==0X10)                             //PID1
    {								
			  memcpy(RatePID,&data_buf[4],18);          //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�
				ANTO_Recived_flag.PID1 = 1;              //���յ���λ��������PID���� �任��־			
        ANO_DT_Send_Check(*(data_buf+2),sum);    //����У����				
    }
    else if(*(data_buf+2)==0X11)                             //PID2
    {
				memcpy(AnglePID,&data_buf[4],18);				 //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�	
			  ANTO_Recived_flag.PID2 = 1;						  //���յ���λ��������PID����  �任��־
        ANO_DT_Send_Check(*(data_buf+2),sum);   //����У����
    }
    else if(*(data_buf+2)==0X12)                             //PID3
    {    
				memcpy(HighPID,&data_buf[4],18);				 //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�	
				ANTO_Recived_flag.PID3 = 1;						  //���յ���λ��������PID���� �任��־
        ANO_DT_Send_Check(*(data_buf+2),sum);   //����У����				       
    }
    else if(*(data_buf+2)==0X13)                             //PID4
    {
    
			memcpy(HighPID4,&data_buf[4],18);				 //�Ȱѽ��յ����������������ֹ����һ��PID���ݸ��ǣ������PID�Ǹ��ٶȻ��õ�	
			ANTO_Recived_flag.PID4 = 1;						  //���յ���λ��������PID���� �任��־
			ANO_DT_Send_Check(*(data_buf+2),sum);

    }
    else if(*(data_buf+2)==0X14)                             //PID5
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
    }
    else if(*(data_buf+2)==0X15)                             //PID6
    {
        ANO_DT_Send_Check(*(data_buf+2),sum);
//        Sort_PID_Cnt++;
//        Sort_PID_Flag=1;
    }
		else  if(*(data_buf+2)==0XF1)  
		{

//			set_flag = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
//		
////			if((set_flag&BIT0) && SPL_Err==0)	keep_high_mode=1;	else keep_high_mode=0;
////			if(set_flag&BIT1)	Throw_Fly_Mode=1; else Throw_Fly_Mode=0;
////			if(set_flag&BIT2)	No_Head_Mode=1; else No_Head_Mode=0;
////			if(set_flag&BIT3)	turn_head=-1; else turn_head=1;
////		
////			if(set_flag&BIT4 && Locat_Err==0 || set_flag&BIT5 && Flow_Err==0)	
////						Fixed_Point_Mode=1;
////			else	Fixed_Point_Mode=0;		  								
////			LED_warn = 2;
		
		}
		else{;}

}



/************************END OF FILE********************/
