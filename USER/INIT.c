//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 810149456
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2018.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ�� ����ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DEFINE.h" 
#include "ALL_DATA.h" 
#include "LED.h"
#include "Uart1.h"
#include "USART2.h"
#include "ANO_Data_Transfer.h"
#include "ADC.h"

volatile uint32_t SysTick_count; //ϵͳʱ�����
volatile uint8_t spl_flag; //ϵͳʱ�����
_st_Mpu MPU6050;   //MPU6050ԭʼ����
_st_Mag AK8975;   
_st_AngE Angle;    //��ǰ�Ƕ���ֵ̬
_st_Remote Remote; //ң��ͨ��ֵ


volatile uint32_t ST_CpuID;
 
 
_st_ALL_flag ALL_flag; //ϵͳ��־λ������������־λ��



 _st_FlightData FlightData;
 //�ɿ�����
st_Command Command;

PidObject pidRateX; //�ڻ�PID����
PidObject pidRateY;
PidObject pidRateZ;

PidObject pidPitch; //�⻷PID����
PidObject pidRoll;
PidObject pidYaw;

PidObject pidHeightRate;
PidObject pidHeightHigh;

PidObject Flow_PosPid_x;    //�⻷����
PidObject Flow_PosPid_y;

PidObject Flow_SpeedPid_x;  //�ڻ�����
PidObject Flow_SpeedPid_y;

_st_IMU IMU;

void pid_param_Init(void); //PID���Ʋ�����ʼ������дPID�����ᱣ�����ݣ��������ɺ�ֱ���ڳ�������� ����¼���ɿ�


//��ȡCPU��ID
void GetLockCode(void)
{
	ST_CpuID = *(vu32*)(0x1ffff7e8);//���ֽ�оƬID������ͨѶ��Ƶͨ��
}

///////////////ȫ����ʼ��//////////////////////////////////
void ALL_Init(void)
{
//	float STBy;

	IIC_Init();             //I2C��ʼ��
		
	pid_param_Init();       //PID������ʼ��
	  
	LEDInit();              //LED���Ƴ�ʼ��

	MpuInit();              //MPU6050��ʼ��
	
		//ADC��ʼ��
//	ADC1_Init();
	
	ANO_Uart1_Init(19200);   //���Դ��� 
//	printf("ANO_Uart1_Init  \r\n")	
	if (FLY_TYPE == 2) 
	{UART2_Init(115200); }      //
	
	USART3_Config(500000);        //��λ�����ڳ�ʼ��
//	printf("USART3_Config  \r\n");

	NRF24L01_init();				//2.4Gң��ͨ�ų�ʼ��
	
	spl_flag=0;
	SPL_Err = 1;
	
	TIM2_PWM_Config();			//2·PWM��ʼ��		
	TIM3_PWM_Config();			//2·PWM��ʼ��		
}

void pid_param_Init(void)
{
		
//////////////////�ڻ��ٶ�PID///////////////////////	
	
	pidRateX.kp = 1.7f;
	pidRateY.kp = 1.7f;
	pidRateZ.kp = 3.0f;
	
	pidRateX.ki = 0.0f;
	pidRateY.ki = 0.0f;
	pidRateZ.ki = 0.0f;	
	
	pidRateX.kd = 0.08f;
	pidRateY.kd = 0.08f;
	pidRateZ.kd = 0.5f;	
	
/////////////�⻷�Ƕ�PID///////////////////////////
	
	pidPitch.kp = 7.0f;
	pidRoll.kp = 7.0f;
	pidYaw.kp = 7.0f;	
	
	pidPitch.ki = 0.0f;
	pidRoll.ki = 0.0f;
	pidYaw.ki = 0.0f;	
	
	pidPitch.kd = 0.0f;
	pidRoll.kd = 0.0f;
	pidYaw.kd = 0.0f;	
	

		//�ڻ�PID���� �ٶ�
	pidHeightRate.kp = 0.0f; //
	pidHeightRate.ki = 0.0f;
	pidHeightRate.kd = 0.0f;
		//�⻷PID����
	pidHeightHigh.kp = 0.0f;//
	pidHeightHigh.ki = 0.0f;
	pidHeightHigh.kd = 0.0f;//
	
	
/////////////////////////////////////////////////////////////////////

	//X�ڻ�����PID����  �ٶ�
	
	Flow_SpeedPid_x.kp = 0.0f;//����  
	Flow_SpeedPid_x.ki = 0.0f;//����
	Flow_SpeedPid_x.kd = 0.0f;//΢��
	
	//X�⻷����PID����  λ��
	
	Flow_PosPid_x.kp = 0.0f;//���� 
	Flow_PosPid_x.ki = 0.0f;//����
	Flow_PosPid_x.kd = 0.0f;//΢��
	
	//////////////////////////////////////////////////////////
	
		//Y�ڻ�����PID���� �ٶ�
	
	Flow_SpeedPid_y.kp = 0.0f;//����
	Flow_SpeedPid_y.ki = 0.0f;//����
	Flow_SpeedPid_y.kd = 0.0f;//΢��
	
	//Y�⻷����PID���� λ�� 
	
	Flow_PosPid_y.kp = 0.0f;//����
	Flow_PosPid_y.ki = 0.0f;//����
	Flow_PosPid_y.kd = 0.0f;//΢��
	

	Command.FlightMode = NORMOL;  //��ʼ��Ϊ��̬����ģʽ
}










