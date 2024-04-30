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
#include "misc.h"
#include "delay.h"
#include "ALL_DATA.h"
#include "scheduler.h"
static volatile uint32_t usTicks = 0;

volatile uint32_t SysTick_count22; //ϵͳ�δ�ʱ��
u8 sys_init_ok = 1;
// �δ�ʱ���������� ,49������
//volatile uint32_t sysTickUptime = 0;
void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;
}
void SysTick_IRQ(void)//1ms�ж�
{
	static u8 cnt;
	
		SysTick_count++;
	if(!sys_init_ok) return;
	
	
	cnt++;	cnt %= 2;
	if(cnt)	Loop_check();//2ms�ж�
}  

uint32_t GetSysTime_us(void) 
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = SysTick_count;
        cycle_cnt = SysTick->VAL;
    	} while (ms != SysTick_count);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

//    ���뼶��ʱ����	 
void delay_ms(uint16_t nms)
{
	uint32_t t0=GetSysTime_us();
	while(GetSysTime_us() - t0 < nms * 1000);	
	SysTick_count22++;
}

void delay_us(unsigned int i)
 {  
	char x=0;   
    while( i--)
    {	
       for(x=1;x>0;x--);
    }
 }	

/**************************************************************
 *����ϵͳ��ǰ������ʱ��  ��λms
 * @param[in] 
 * @param[out] 
 * @return  ms   
 ***************************************************************/	
float micros(void) //����ϵͳ��ǰʱ��
{
	 //SysTick_count �δ�ʱ��ms����
	 //SysTick->VAL �δ�ʱ�Ӷ�ʱ��������counter �ܼ�����168000.0f/8.0fΪ1��ms���������δ�ʱ��ms�ж�
	 //168MHZ/1000 = 168000ÿ��ms��Ҫ�ľ���Ƶ������
	 //�δ�ʱ�Ӱ˷�Ƶ
    return SysTick_count22 + (SysTick->LOAD - SysTick->VAL)/(72000.0f/8.0f);//��ǰϵͳʱ�� ��λms  
}
///***********************************************************************
// * 
// * @param[in] 
// * @param[out] 
// * @return     
// **********************************************************************/
//float height;

/********************************END OF FILE*****************************************/ 
