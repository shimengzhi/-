//========================================================================
//	爱好者电子工作室-淘宝 https://devotee.taobao.com/
//	STM32四轴爱好者QQ群: 799870988
//	作者：小刘
//	电话:13728698082
//	邮箱:1042763631@qq.com
//	日期：2020.05.17
//	版本：V1.0
//========================================================================
//套件购买地址：https://devotee.taobao.com/
//                 爱好者电子工作室
//特此声明：
//
//         此程序只能用作学习，如用商业用途。必追究责任！
//          
//
//
#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "ANO_Data_Transfer.h"

#include "STM32F10x_IWDG.h"

//特别声明：请在无人空旷的地带或者室内进行飞行。遇到非常紧急的情况，可紧急关闭遥控。
/*初始化独立看门狗
//prer:分频数:0~7(只有低3位有效！)
//分频因子=4*2^prer.最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).   */

void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写超作
	
	IWDG_SetPrescaler(prer);  //设置IWDG预分频值：设置IWDG预分频值64
	
	IWDG_SetReload(rlr);  //设置IWDG重装载值
	
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable();  //使能IWDG
}
 
//喂独立看门狗
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//喂狗								   
}

int main(void)
{	
	cycleCounterInit();  //得到系统每个us的系统CLK个数，为以后延时函数，和得到精准的当前执行时间使用
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //4个bit的抢占优先级，4个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);	//系统滴答时钟

	ALL_Init();//系统初始化 
	
  IWDG_Init(4,625);    //初始化看门狗  与分频为64,重载值为625,溢出时间1s
	
	while(1)
	{
		  main_loop();  //程序运行
		
			IWDG_Feed();  //喂看门狗防止程序跑死
	}
}










