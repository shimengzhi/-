//========================================================================
//	爱好者电子工作室-淘宝 https://devotee.taobao.com/
//	STM32四轴爱好者QQ群: 810149456
//	作者：小刘
//	电话:13728698082
//	邮箱:1042763631@qq.com
//	日期：2018.05.17
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

#include "spi.h"
#include "sys.h"

void SPI_Config(void)
{
      SPI_InitTypeDef  SPI_InitStructure;
   	  GPIO_InitTypeDef GPIO_InitStructure;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);  
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO  , ENABLE);
			GPIO_SetBits(NRF_CSN_GP, NRF24L01_CSN); //NRF_CS预置为高 
		
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

			/* SPI2 configuration */
			SPI_Cmd(SPI2, DISABLE);             //必须先禁能,才能改变MODE
			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
			SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
			SPI_InitStructure.SPI_CRCPolynomial = 7;

			SPI_Init(SPI2, &SPI_InitStructure);

			/* SPI2 enable */
			SPI_Cmd(SPI2, ENABLE);
}
u8 SPI_RW(u8 dat)
{
        /* Loop while DR register in not emplty */
        while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

        /* Send byte through the SPI2 peripheral */
        SPI_I2S_SendData(SPI2, dat);

        /* Wait to receive a byte */
        while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

        /* Return the byte read from the SPI bus */
        return SPI_I2S_ReceiveData(SPI2);  
}
