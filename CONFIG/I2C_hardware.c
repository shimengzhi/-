#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "I2C.h"

#define I2C1_OWN_ADDRESS7 0xcc
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC��ʼ��
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:           IIC��
  *           SlaveAdd:       ��Ϊ���豸ʱʶ���ַ
  *           F103IIC1_Remap: ���F103��IIC1�Ƿ���ӳ��
  *                           0: ����ӳ��
  *                           1: ��ӳ�䡣PB.06 -> PB.08��PB.07 -> PB.09
  *
  * @Output : none
  *
  * @Return : none
  *****************************************************************************
**/
void Hard_IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//
	//�ܽŸ���
	//
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);
		
	//
	//����IIC����
	//
	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0xcc;  //���豸��ַ
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = (400 * 1000);  //SCL���100KHz
	
	
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_AcknowledgeConfig(I2C1, ENABLE); 
	
	//
	//��ֹʱ���ӳ�
	//
	I2C1->CR1 |= 1<<7;
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC�ȴ����豸�ڲ��������
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:  ��ȡ��EEPROM�ڴ��ַ
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
void Hard_IICWaiteStandby(uint8_t SlaveAdd)
{
	u16 tmp = 0;
	
	I2C1->SR1 &= 0x0000;  //���״̬�Ĵ���1
	
	do
	{
		I2C_GenerateSTART(I2C1, ENABLE);  //������ʼ�ź�
		tmp = I2C1->SR1;  //��ȡSR1�Ĵ�����Ȼ��д�����ݼĴ������������SBλ��EV5
		I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Transmitter);  //���ʹ��豸��ַ
	} while ((I2C1->SR1 & 0x0002) == 0x0000);  //��ADDR = 1ʱ������Ӧ���ˣ�����ѭ��
	I2C_ClearFlag(I2C1, I2C_FLAG_AF);  //���Ӧ��ʧ�ܱ�־λ
	I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC����һ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           WriteAdd: д��EEPROM�ڴ��ַ
  *           Data:     д�������
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : none
  *****************************************************************************
**/
u8 Hard_IICWriteOneByte(uint8_t SlaveAdd, u8 WriteAdd, u8 Data)
{
	u16 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	
	I2C_GenerateSTART(I2C1, ENABLE);  //������ʼ�ź�
	temp = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	
	I2C_Send7bitAddress(I2C1,SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	temp = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		temp++;
		if (temp > 1000)
		{
			I2C1->CR1 |= I2C_CR1_STOP;  //����ֹͣ�ź�
			return FAILED;
		}
	}
	//
	//��ȡSR2״̬�Ĵ���
	//
	flag = I2C1->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C1->DR = WriteAdd;  //���ʹ洢��ַ
	temp = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_SendData(I2C1, Data);  //��������
	temp = 0;
	//
	//EV8_2
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
	return SUCCESS;
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC��ȡһ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:     IIC��
  *           SlaveAdd: ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:  ��ȡ��EEPROM�ڴ��ַ
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
u8 Hard_IIC_ReadOneByte(uint8_t SlaveAdd, u8 ReadAdd,u8 *REG_data)
{
	u16 i = 0;
	u8 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTART(I2C1, ENABLE);  //������ʼ�ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	flag = I2C1->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C_SendData(I2C1, ReadAdd);  //���ʹ洢��ַ
	i = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		i++;
		if (i > 2000)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);  //�����ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{			
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Receiver);  //��ȡ����
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	flag = I2C1->SR2;
	
	I2C_AcknowledgeConfig(I2C1, DISABLE);  //����NACK
	//I2C_GenerateSTOP(I2C1, ENABLE);
	i = 0;
	//
	//EV7
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	*REG_data = I2C_ReceiveData(I2C1);
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
	return SUCCESS;
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC���Ͷ���ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:       IIC��
  *           SlaveAdd:   ��Ϊ���豸ʱʶ���ַ
  *           WriteAdd:   д��EEPROM�ڴ���ʼ��ַ
  *           NumToWrite: д��������
  *           *pBuffer:   д��������黺��
  *
  * @Output : *err:     ���صĴ���ֵ
  *
  * @Return : none
  *****************************************************************************
**/
u8 Hard_IIC_WriteNByte(u8 SlaveAdd, u8 WriteAdd, u8 NumToWrite, u8 * pBuffer)
{
	u16 sta = 0;
	u16 temp = 0;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTART(I2C1, ENABLE);  //������ʼ�ź�
	temp = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	temp = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		temp++;
		if (temp > 1000)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	//
	//��ȡSR2״̬�Ĵ���
	//
	sta = I2C1->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	I2C_SendData(I2C1, WriteAdd);  //���ʹ洢��ַ
	temp = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		temp++;
		if (temp > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED ;
		}
	}
	//
	//ѭ����������
	//
	while (NumToWrite--)
	{
		I2C_SendData(I2C1, *pBuffer);  //��������
		pBuffer++;
		temp = 0;
		//
		//EV8_2
		//
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		{
			temp++;
			if (temp > 800)
			{
				I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
				return FAILED;
			}
		}
	}
	I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
	
	return SUCCESS;
}

/**
  *****************************************************************************
  * @Name   : Ӳ��IIC��ȡ����ֽ�����
  *
  * @Brief  : none
  *
  * @Input  : I2Cx:      IIC��
  *           SlaveAdd:  ��Ϊ���豸ʱʶ���ַ
  *           ReadAdd:   ��ȡ��EEPROM�ڴ���ʼ��ַ
  *           NumToRead: ��ȡ����
  *
  * @Output : *pBuffer: �������������
  *           *err:     ���صĴ���ֵ
  *
  * @Return : ��ȡ��������
  *****************************************************************************
**/
u8 Hard_IIC_ReadNByte(u8 SlaveAdd, u8 ReadAdd,u8 NumToWrite,u8 * pBuffer)
{
	u16 i = 0;
	u8 temp = 0;
	u16 flag = 0;
	
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))  //�ȴ�IIC
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTART(I2C1, ENABLE);  //������ʼ�ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Transmitter);  //�����豸��ַ
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	flag = I2C1->SR2;  //�����ȡSR1�Ĵ�����,��SR2�Ĵ����Ķ����������ADDRλ�������٣�����������������
	
	I2C_SendData(I2C1, ReadAdd);  //���ʹ洢��ַ
	i = 0;
	//
	//EV8
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING))
	{
		i++;
		if (i > 2000)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_GenerateSTOP(I2C1, ENABLE);
	I2C_GenerateSTART(I2C1, ENABLE);  //�����ź�
	i = 0;
	//
	//EV5
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
	{
		i++;
		if (i > 800)
		{			
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	I2C_Send7bitAddress(I2C1, SlaveAdd, I2C_Direction_Receiver);  //��ȡ����
	i = 0;
	//
	//EV6
	//
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		i++;
		if (i > 800)
		{
			I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
			return FAILED;
		}
	}
	flag = I2C1->SR2;
	I2C_AcknowledgeConfig(I2C1, ENABLE);  //����NACK	
	
	while (NumToWrite)
	{
		i = 0;	

		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
		{
			i++;
			if (i > 800)
			{
				I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
				return FAILED;
			}
		}
        *pBuffer = I2C_ReceiveData(I2C1);	
        if (NumToWrite == 1)
            I2C_AcknowledgeConfig(I2C1, DISABLE);  //����NACK
        else
            I2C_AcknowledgeConfig(I2C1, ENABLE);  //����NACK		
        pBuffer++;
        NumToWrite--;
    }
		I2C_GenerateSTOP(I2C1, ENABLE);  //����ֹͣ�ź�
		return SUCCESS;
}



 
 
 
 
 
 
 
 
