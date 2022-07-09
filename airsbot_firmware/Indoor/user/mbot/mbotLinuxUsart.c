#include "mbotLinuxUsart.h"
#include "usart.h"         //����printf
#include "stm32f10x_usart.h"

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/

/*--------------------------------����Э��-----------------------------------
//----------------55 aa size 00 00 00 00 00 crc8 0d 0a----------------------
//����ͷ55aa + �����ֽ���size + ���ݣ����ù����壩 + У��crc8 + ����β0d0a
//ע�⣺����������Ԥ����һ���ֽڵĿ���λ�������Ŀ���������չ������size������
--------------------------------------------------------------------------*/


/**************************************************************************
ͨ�ŵķ��ͺ����ͽ��պ��������һЩ���������������������
**************************************************************************/

//���ݽ����ݴ���
unsigned char  receiveBuff[16] = {0};         
//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
union sendData
{
	short d;
	unsigned char data[2];
}leftVelNow,rightVelNow,imu1,imu2,imu3,imu4,imu5,imu6,imu7,imu8,imu9,imu10;

//�������ٿ����ٶȹ�����
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;

/**************************************************************************
�������ܣ�ͨ�������жϷ���������ȡ��λ�����͵������ֿ����ٶȡ�Ԥ�����Ʊ�־λ���ֱ���������
��ڲ������������ٿ��Ƶ�ַ���������ٿ��Ƶ�ַ��Ԥ�����Ʊ�־λ
����  ֵ������������
**************************************************************************/
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //��������
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART2);   //@@@@@#####�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�����������ٶ����ݵĳ���
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://����У��ֵ��Ϣ
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // �����ϢУ��ֵ
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
				//	printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[9]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[10] �����ж�

					//�����ٶȸ�ֵ����					
					 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						rightVelSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
					}				
					
					//�ٶȸ�ֵ����
					*p_leftSpeedSet  = (int)leftVelSet.d;
					*p_rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}
/**************************************************************************
�������ܣ����������ٺͽǶ����ݡ������źŽ��д����ͨ�����ڷ��͸�Linux
��ڲ�����ʵʱ�������١�ʵʱ�������١�ʵʱ�Ƕȡ������źţ����û�нǶ�Ҳ���Բ�����
����  ֵ����
**************************************************************************/
void usartSendData(short leftVel, short rightVel,short imu_1,short imu_2,short imu_3,short imu_4,short imu_5,short imu_6,short imu_7,short imu_8,short imu_9,short imu_10,unsigned char ctrlFlag)
{
	// Э�����ݻ�������
	unsigned char buf[31] = {0};
	int i, length = 0;

	// ���������������ٶ�
	leftVelNow.d  = leftVel;
	rightVelNow.d = rightVel;
	imu1.d    = imu_1;
	imu2.d    = imu_2;
	imu3.d    = imu_3;
	imu4.d    = imu_4;
	imu5.d    = imu_5;
	imu6.d    = imu_6;
	imu7.d    = imu_7;
	imu8.d    = imu_8;
	imu9.d    = imu_9;
	imu10.d    = imu_10;
	
	// ������Ϣͷ
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1] 
	
	// ���û������������ٶȡ��Ƕ�
	length = 25;
	buf[2] = length;                             // buf[2]
	for(i = 0; i < 2; i++)
	{
		buf[i + 3] = leftVelNow.data[i];         // buf[3] buf[4]
		buf[i + 5] = rightVelNow.data[i];        // buf[5] buf[6]
		buf[i + 7] = imu1.data[i];           // buf[7] buf[8]
		buf[i + 9] = imu2.data[i];         // buf[3] buf[4]
		buf[i + 11] =imu3.data[i];        // buf[5] buf[6]
		buf[i + 13] =imu4.data[i];           // buf[7] buf[8]
		buf[i + 15] =imu5.data[i];         // buf[3] buf[4]
		buf[i + 17] =imu6.data[i];        // buf[5] buf[6]
		buf[i + 19] =imu7.data[i];           // buf[7] buf[8]
		buf[i + 21] =imu8.data[i];         // buf[3] buf[4]
    buf[i + 23] =imu9.data[i];           // buf[7] buf[8]
		buf[i + 25] =imu10.data[i];       
	}
	// Ԥ������ָ��
	buf[3 + length - 1] = ctrlFlag;              // buf[9]
	
	// ����У��ֵ����Ϣβ
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[10]
	buf[3 + length + 1] = ender[0];              // buf[11]
	buf[3 + length + 2] = ender[1];              // buf[12]
	
	//�����ַ�������
	USART_Send_String(buf,sizeof(buf));
}
/**************************************************************************
�������ܣ�����ָ����С���ַ����飬��usartSendData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		//@@@@@#####�����ʹ�ò���USART1���ĳ���Ӧ�ģ�����USART3�������������޸�
		while( !(USART2->SR&(0x01<<7)) );//���ͻ�����Ϊ��
		USART2->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}
/**************************************************************************
�������ܣ������λѭ������У�飬��usartSendData��usartReceiveOneData��������
��ڲ����������ַ�������С
����  ֵ����
**************************************************************************/
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}
/**********************************END***************************************/







