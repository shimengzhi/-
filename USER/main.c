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
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "ANO_Data_Transfer.h"

#include "STM32F10x_IWDG.h"

//�ر��������������˿տ��ĵش��������ڽ��з��С������ǳ�������������ɽ����ر�ң�ء�
/*��ʼ���������Ź�
//prer:��Ƶ��:0~7(ֻ�е�3λ��Ч��)
//��Ƶ����=4*2^prer.���ֵֻ����256!
//rlr:��װ�ؼĴ���ֵ:��11λ��Ч.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/40 (ms).   */

void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
	
	IWDG_SetPrescaler(prer);  //����IWDGԤ��Ƶֵ������IWDGԤ��Ƶֵ64
	
	IWDG_SetReload(rlr);  //����IWDG��װ��ֵ
	
	IWDG_ReloadCounter();  //����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
	
	IWDG_Enable();  //ʹ��IWDG
}
 
//ι�������Ź�
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//ι��								   
}

int main(void)
{	
	cycleCounterInit();  //�õ�ϵͳÿ��us��ϵͳCLK������Ϊ�Ժ���ʱ�������͵õ���׼�ĵ�ǰִ��ʱ��ʹ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //4��bit����ռ���ȼ���4��bit�������ȼ�
	SysTick_Config(SystemCoreClock / 1000);	//ϵͳ�δ�ʱ��

	ALL_Init();//ϵͳ��ʼ�� 
	
  IWDG_Init(4,625);    //��ʼ�����Ź�  ���ƵΪ64,����ֵΪ625,���ʱ��1s
	
	while(1)
	{
		  main_loop();  //��������
		
			IWDG_Feed();  //ι���Ź���ֹ��������
	}
}










