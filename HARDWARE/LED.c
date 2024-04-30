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
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "stm32f10x.h"
#include "LED.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h" 
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
u8 LED_warn;                             //default 300ms flash the status
/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
		GPIO_InitTypeDef GPIO_InitStructure;
		AFIO->MAPR = 0X02000000; //ʹ��4����д �ͷ�ĳЩ����д��ص�����
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = fLED_io | hLED_io|aLED_io | bLED_io;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(LED_GPIOB, &GPIO_InitStructure); 	 
	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;

	if(SysTick_count - LastTime < LED.FlashTime)
	{

		return;
	}
	else
		LastTime = SysTick_count;
	switch(LED.status)
	{
		case AlwaysOff:      //����   
			bLED_H();
			fLED_H();
			bLED_H();
			hLED_H();
			break;
		case AllFlashLight:				  //ȫ��ͬʱ��˸
			fLED_Toggle();			
			bLED_Toggle();
			hLED_Toggle();			
			aLED_Toggle();		
		  break;
		case AlwaysOn:  //����
			
		  bLED_L();
			fLED_L();
			aLED_L();
			hLED_L();
		  break;
		case AlternateFlash: //������˸
			bLED_H();
			fLED_L();
			aLED_H();
			hLED_L();
			LED.status = AllFlashLight;
			break;
		case WARNING:
		  fLED_Toggle();
		  hLED_Toggle();
			bLED_Toggle();
		  aLED_Toggle();
			LED.FlashTime = 800;
			break;
		case DANGEROURS:
			bLED_L();
		  aLED_L();
			fLED_Toggle();
		  hLED_Toggle();
			LED.FlashTime = 70;
			break;
		default:
			LED.status = AlwaysOff;
			break;
	}
}

/**************************END OF FILE*********************************/



