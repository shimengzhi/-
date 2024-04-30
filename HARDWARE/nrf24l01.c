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
#include "ALL_DATA.h"
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "LED.h"
#include "ANO_DT.h"
#include "sys.h"

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR				6		//
#define TX_DS				5
#define MAX_RT			4


u8 MPU_Err=1,NRF_Err=1,SPL_Err=1;
uint8_t NRF_SSI,NRF_SSI_CNT;//NRF�ź�ǿ��
uint16_t Nrf_Erro;


u8 _CH;
uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
uint8_t NRF24L01_2_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����

//�������
const uint8_t TX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	//���ص�ַ
const uint8_t RX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	//���յ�ַRX_ADDR_P0 == RX_ADDR

//////////////////////////////////////////////////////////////////////////////////////////////////////////
	


	//24L01������   
	#define Set_NRF24L01_CSN    (NRF_CSN_GP->BSRR = NRF24L01_CSN)  // 
	#define Clr_NRF24L01_CSN    (NRF_CSN_GP->BRR = NRF24L01_CSN)   //  
	#define Set_NRF24L01_CE 		(NRF_CE_GP->BSRR = NRF24L01_CE)    // 
	#define Clr_NRF24L01_CE  		(NRF_CE_GP->BRR = NRF24L01_CE)     //  
	#define READ_NRF24L01_IRQ   (NRF_IRQ_GP->IDR&NRF24L01_IRQ)     //IRQ������������ 

//��ʼ��24L01��IO��
void NRF24L01_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);    //ʹ��GPIO��ʱ��  CE
	
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CE;          //NRF24L01  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NRF_CE_GP, &GPIO_InitStructure);

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //ʹ��GPIO��ʱ�� CSN
	GPIO_InitStructure.GPIO_Pin = NRF24L01_CSN;      
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NRF_CSN_GP, &GPIO_InitStructure); 
	
	Set_NRF24L01_CE;                                    //��ʼ��ʱ������
	Set_NRF24L01_CSN;                                   //��ʼ��ʱ������

    //����NRF2401��IRQ
	GPIO_InitStructure.GPIO_Pin = NRF24L01_IRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU  ;     //��������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NRF_IRQ_GP, &GPIO_InitStructure);
	
//	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	SPI_Config();                                //��ʼ��SPI
	Clr_NRF24L01_CE; 	                                //ʹ��24L01
	Set_NRF24L01_CSN;                                   //SPIƬѡȡ��
}
//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ�
//��ͬʱ����ֵ:0����ʾ��λ;���򷵻�1����ʾ����λ	
u8 NRF24L01_Check(void)
{
	u8 buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	u8 buf1[5];
	u8 i;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //����д��ĵ�ַ  	
	for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;					   
	if(i!=5)return 1;                               //NRF24L01����λ	
	return 0;		                                    //NRF24L01��λ
}	 	 
//ͨ��SPIд�Ĵ���
u8 NRF24L01_Write_Reg(u8 regaddr,u8 data)
{
	u8 status;	
    Clr_NRF24L01_CSN;                    //ʹ��SPI����
  	status =SPI_RW(regaddr); //���ͼĴ����� 
  	SPI_RW(data);            //д��Ĵ�����ֵ
  	Set_NRF24L01_CSN;                    //��ֹSPI����	   
  	return(status);       		         //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
u8 NRF24L01_Read_Reg(u8 regaddr)
{
	u8 reg_val;	    
	Clr_NRF24L01_CSN;                //ʹ��SPI����		
  	SPI_RW(regaddr);     //���ͼĴ�����
  	reg_val=SPI_RW(0XFF);//��ȡ�Ĵ�������
  	Set_NRF24L01_CSN;                //��ֹSPI����		    
  	return(reg_val);                 //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
u8 NRF24L01_Read_Buf(u8 regaddr,u8 *pBuf,u8 datalen)
{
	u8 status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //ʹ��SPI����
  	status=SPI_RW(regaddr);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_RW(0XFF);//��������
  	Set_NRF24L01_CSN;                     //�ر�SPI����
  	return status;                        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
u8 NRF24L01_Write_Buf(u8 regaddr, u8 *pBuf, u8 datalen)
{
	u8 status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                    //ʹ��SPI����
  	status = SPI_RW(regaddr);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_RW(*pBuf++); //д������	 
  	Set_NRF24L01_CSN;                                    //�ر�SPI����
  	return status;                                       //���ض�����״ֵ̬
}				   
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
u8 NRF24L01_TxPacket(u8 *txbuf)
{
	u8 state;   
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
	Set_NRF24L01_CE;                                     //��������	   
	while(READ_NRF24L01_IRQ!=0);                         //�ȴ��������
	state=NRF24L01_Read_Reg(STATUS);                     //��ȡ״̬�Ĵ�����ֵ	   
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //���TX_DS��MAX_RT�жϱ�־
	if(state&MAX_TX)                                     //�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //���TX FIFO�Ĵ��� 
		return MAX_TX; 
	}
	if(state&TX_OK)                                      //�������
	{
		return SUCCESS;
	}
	return FAILED;                                         //����ԭ����ʧ��
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)                                 //���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ��� 
		return SUCCESS; 
	}	   
	return FAILED;                                      //û�յ��κ�����
}

//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
void RX_Mode(void)
{
	Clr_NRF24L01_CE;	  
    //дRX�ڵ��ַ
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);

    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    
    //ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
	  //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,45);	  //(Ҫ�ĵ������ƣ��Ͱ�45�ĳɸ�ң��������һ���ġ��Ϳ��Ե���������)
    //ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
    //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    //CEΪ��,�������ģʽ 
  	Set_NRF24L01_CE;                                
}			

//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,
//ѡ��RFƵ��,�����ʺ�LNA HCURR PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.	 
void TX_Mode(void)
{														 
	Clr_NRF24L01_CE;	    
    //дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);    
    //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH); 

    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);     
    //ʹ��ͨ��0�Ľ��յ�ַ  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); 
    //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
    //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,45);       //(Ҫ�ĵ������ƣ��Ͱ�45�ĳɸ�ң��������һ���ġ��Ϳ��Ե���������)
    //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //0x27  250K   0x07 1M
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ,���������ж�
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // CEΪ��,10us����������
	Set_NRF24L01_CE;                                  
}	

void ANO_NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
	Clr_NRF24L01_CE;		 //StandBy Iģʽ
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 //װ������
	Set_NRF24L01_CE;  
}

void ANO_NRF_Init(u8 model, u8 ch)
{
	Clr_NRF24L01_CE;
	
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(u8*)RX_ADDRESS,RX_ADR_WIDTH);	//дRX�ڵ��ַ
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH); 		//дTX�ڵ��ַ 
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01); 															//ʹ��ͨ��0���Զ�Ӧ��
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);														//ʹ��ͨ��0�Ľ��յ�ַ
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);													//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 2M��������
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ch);																	//����RFͨ��ΪCHANAL
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); 														//����TX�������,0db����,2Mbps,���������濪��
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												    			//����TX�������,0db����,1Mbps,���������濪��
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x27); 												   				//����TX�������,0db����,250Kbps,���������濪��
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		        					// IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 									// IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		  								// IRQ�շ�����жϿ���,16λCRC,������
		
		SPI_RW(0x50);
		SPI_RW(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   											// IRQ�շ�����жϿ���,16λCRC,������
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		
		SPI_RW(0x50);
		SPI_RW(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
		Set_NRF24L01_CE;   
}
void NRF24L01_init(void)
{
	NRF24L01_Configuration();
	
	bLED_H();	 	//ǰ����
	aLED_H();		//ǰ����
	fLED_L();		//�����
	hLED_L();		//�����
  do
	{ 
		GetLockCode();
		_CH = ST_CpuID % 0x7E;      //оƬID������ͨѶ��Ƶͨ��
		ANO_NRF_Init(MODEL_RX2,0);
		NRF_Err=1;
	}while(NRF24L01_Check() == FAILED);
	NRF_Err=0;
}
//////////////////////////////////////////////////////////////


void ANO_NRF_Check_Event(void)
{
	u8 sta = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);   //�����ձ�־
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	//printf("sta: %d   \r\n",sta);  //����1���
	if(sta & (1<<RX_DR))																	//�ж��Ƿ��յ�����
	{
		u8 rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);       
		if(rx_len<33)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_2_RXDATA,rx_len); //��������
			Nrf_Erro = 0;
		}
		else 
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);//���������
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<TX_DS))
	{
		
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, sta);
}

u8 NRF_Connect(void)//1KHZ
{
	static u8 Connect_flag;
	
	Nrf_Erro ++;
	if(Nrf_Erro==1)
	{
		ANO_DT_Data_Receive_Anl(NRF24L01_2_RXDATA,NRF24L01_2_RXDATA[3]+5);//����2.4G����
		NRF_SSI_CNT++;
	//	printf("NRF_SSI_CNT: %d   \r\n",NRF_SSI_CNT);  //����1���

		Connect_flag = 1;
	}
	if(Nrf_Erro>=500)
	{
		Nrf_Erro = 1;
		Connect_flag = 0;
	}
	return Connect_flag;
}



/*********************END OF FILE******************************************************/





















/*********************END OF FILE******************************************************/
















