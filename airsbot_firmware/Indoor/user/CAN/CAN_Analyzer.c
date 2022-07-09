
#include "stm32f10x_tim.h"
#include "misc.h"
#include"CAN_Analyzer.h"
#include"COM_CMD.h"
#include <stdio.h>
#include<string.h>

CAN_Data change_toshow[CAN_TBUFFLEN];  //变化次数 用于显示常变数据	Tick为次数
CAN_Data change_tocount[CAN_TBUFFLEN];  // 变化次数 用于计算  Tick为次数

uint8_t SHOW_CMD=0;
uint32_t Set_ID=0xFFFFFFFF;
uint8_t  ANA_PORT=0x00;
uint32_t ANA_ID=0xFFFFFFFF;
uint32_t ANA_MASK=0xFFFFFFFF;

CAN_Data CAN_ALL_ID[CAN_TBUFFLEN];  // 所有ID记录

CAN_T CAN_Time[CAN_TBUFFLEN];   // CAN ID对应ms数
uint32_t CAN_CrossSendCtrl[CAN_CBUFFLEN];  // CAN 转发记录 有ID记录为禁止转发

/*****************************

*****************************/
void Char2Str(char *Datout,char *Datin,unsigned char len)
{
    unsigned char j;
    
    for(j=0;j<len;j++)
    {
        sprintf(Datout,",%02X",Datin[j]);
        Datout+=3;	
    }	
}

/*****************************
    CAN接收的数据入口
*****************************/
void CAN_RX_Data(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim)
{
    uint8_t x=0;
	uint32_t IDtemp;

    if(RxMsg->IDE==CAN_ID_STD)	// 标准帧
	    IDtemp=RxMsg->StdId;
	else  // 扩展帧
	    IDtemp=RxMsg->ExtId;

    for(x=0;x<CAN_TBUFFLEN;x++)  // 查找是否有记录
	{
	    if(IDtemp==change_tocount[x].ID)  // 有记录
		{
		    if(memcmp(RxMsg->Data,change_tocount[x].Data,RxMsg->DLC))  // 比较数据
			{
				// 更新数据
				change_tocount[x].Port=port;  
				change_tocount[x].ID=IDtemp;
				change_tocount[x].RTR=RxMsg->RTR;
				change_tocount[x].IDE=RxMsg->IDE;
				change_tocount[x].DLC=RxMsg->DLC;
				if(change_tocount[x].Tick<10000)  // 次数++
				    change_tocount[x].Tick++;
			    memcpy(change_tocount[x].Data,RxMsg->Data,RxMsg->DLC); 

				break;
			}
		         
		}
		else
		if(change_tocount[x].ID==0xFFFFFFFF) // 查找完，无记录
		{
			// 更新数据
			change_tocount[x].Port=port;  
			change_tocount[x].ID=IDtemp;
			change_tocount[x].RTR=RxMsg->RTR;
			change_tocount[x].IDE=RxMsg->IDE;
			change_tocount[x].DLC=RxMsg->DLC;
			change_tocount[x].Tick=0;   // 次数清0
		    memcpy(change_tocount[x].Data,RxMsg->Data,RxMsg->DLC); 	
			
			break;	
		}
	}

}

/*****************************
    CAN分析初始化
*****************************/
void CAN_AnalyInit(void)
{
	uint8_t k;

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // 打开TIM2时钟	
    TIM_DeInit(TIM2);               //复位TIM2定时器
    
    /* TIM2 configuration 72M/36000/2=1KHz */
    TIM_TimeBaseStructure.TIM_Period = 1;        // 1ms     
    TIM_TimeBaseStructure.TIM_Prescaler = 36000;    // 分频36000       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 
    /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
 
    /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
 
    /* TIM2 enable counter [允许tim2计数]*/
    TIM_Cmd(TIM2, ENABLE); 
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
								  
    // ID对应的ms数列表
	for(k=0;k<CAN_TBUFFLEN;k++)	  
	    CAN_Time[k].ID=0xFFFFFFFF;
	// 禁止转发列表	
    for(k=0;k<CAN_CBUFFLEN;k++)   
	    CAN_CrossSendCtrl[k]=0xFFFFFFFF;   

    // 所有ID
	for(k=0;k<CAN_TBUFFLEN;k++)
	    CAN_ALL_ID[k].ID=0xFFFFFFFF;
}

/*****************************

  向串口发送一条CAN数据 格式：
   [0]   [1]   [2]	  [3]  [4] [5] [6] [7] [8] [9] [10] [11] ... [18] [19] [20]  [21] [22] [23]
  0x5A  0xA5  length  CMD  ID0 ID1 ID2 ID3 IDE RTR DLC      Data 8   tick0 tick1 Port 0x00 Checksum
  length:  [3]-[22]长度
  checksum  [2]+[3]+...[22]
  ----- [22]预留填0x00


  port    CAN端口 1：CAN1  2：CAN2
  RxMsg	  CAN数据
  Tim     时间tick
*****************************/
void Comm_Send_CANmsg(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim)
{
    u8 m=0;	
	u8 buf[30];

	memset(buf,0x00,30);  // 清空

	buf[0] = HEAD_ONE;
	buf[1] = HEAD_TWO;
	buf[2] = 20;       // length
	buf[3] = OUT_CMD_CAN_DATA;  // CMD

	if(RxMsg->IDE==CAN_ID_STD) // 标准帧  
	{
	    buf[4]=(u8) ( RxMsg->StdId&0x000000FF); 
		buf[5]=(u8) ((RxMsg->StdId&0x0000FF00)>>8);
		buf[6]=(u8) ((RxMsg->StdId&0x00FF0000)>>16);
		buf[7]=(u8) ((RxMsg->StdId&0xFF000000)>>24);
	}  
	else  // 扩展帧
	{
	    buf[4]=(u8) ( RxMsg->ExtId&0x000000FF); 
		buf[5]=(u8) ((RxMsg->ExtId&0x0000FF00)>>8);
		buf[6]=(u8) ((RxMsg->ExtId&0x00FF0000)>>16);
		buf[7]=(u8) ((RxMsg->ExtId&0xFF000000)>>24);
	}
	buf[8]=RxMsg->IDE;
	buf[9]=RxMsg->RTR;
	buf[10]=RxMsg->DLC;
  	for(m=0;m<8;m++)
	    buf[m+11]=RxMsg->Data[m];    // Data
	buf[19]= Tim&0x00FF;       // tick0
	buf[20]=(Tim&0xFF00)>>8;   // tick1
	buf[21]= port;    // port
	buf[22]= 0x00;	  // 预留
	for(m=2;m<23;m++)   // checksum
	    buf[23]+=buf[m];

	USART_Send(USART1,buf,24);   // 串口发送
}

/*****************************

  向串口发送一条CAN数据 字符串
  port    CAN端口 1：CAN1  2：CAN2
  RxMsg	  CAN数据
  Tim     时间tick
*****************************/
void Comm_Send_CANmsg_str(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim)
{
    char Buf[60];

	memset(Buf,0x00,60);  // 清空

	sprintf(Buf,"%d",port);    
	USART_STR(USART1,Buf);

	if(RxMsg->IDE==CAN_ID_STD) // 标准帧 
	{
	    sprintf(Buf,",S0x%08X",RxMsg->StdId);
		USART_STR(USART1,Buf);

		sprintf(Buf,",%d",RxMsg->DLC);
	    USART_STR(USART1,Buf);	
	    Char2Str(Buf,(char*)RxMsg->Data,8);
	    USART_STR(USART1,Buf);
	}
	else // 扩展帧
	{
	    sprintf(Buf,",E0x%08X",RxMsg->ExtId);
		USART_STR(USART1,Buf);

		sprintf(Buf,",%d",RxMsg->DLC);
	    USART_STR(USART1,Buf);	
	    Char2Str(Buf,(char*)RxMsg->Data,8);
	    USART_STR(USART1,Buf);
	}
	// tick
	sprintf(Buf,",/%04d",Tim);
	USART_STR(USART1,Buf);
	    
	USART_STR(USART1,"\r\n");	
			
}

/*****************************

  串口发送ID 字符串

  port    CAN端口 1：CAN1  2：CAN2
  RxMsg	  CAN数据
*****************************/
void Comm_Send_ID_str(uint8_t port,CanRxMsg *RxMsg)
{
    u8 m=0;	
	char buf[20];
	u8 static count=0;

    if(RxMsg->IDE==CAN_ID_STD)  // 标准帧
	{
		for(m=0;m<CAN_TBUFFLEN;m++)
		{
		    if(CAN_ALL_ID[m].ID==RxMsg->StdId)  // 有记录
			{
			    break;
			}
			else
			if(CAN_ALL_ID[m].ID==0xFFFFFFFF) // 查找完已有记录，新ID
			{
			    count++;
				// 保存ID
			    CAN_ALL_ID[m].ID=RxMsg->StdId;  
				CAN_ALL_ID[m].Port=port;
				CAN_ALL_ID[m].IDE=CAN_ID_STD;
				// 打印ID
				sprintf(buf,"S 0x%08X",RxMsg->StdId);
				USART_STR(USART1,buf);
				// 打印count
				sprintf(buf," -- %d - %03d\r\n",port,count);  
				USART_STR(USART1,buf);
				break;
			}
		}	
	}
	else  // 扩展帧
	{
		for(m=0;m<CAN_TBUFFLEN;m++)
		{
		    if(CAN_ALL_ID[m].ID==RxMsg->ExtId)  // 有记录
			{
			    break;
			}
			else
			if(CAN_ALL_ID[m].ID==0xFFFFFFFF) // 查找完已有记录，新ID
			{
			    count++;
				// 保存ID
			    CAN_ALL_ID[m].ID=RxMsg->ExtId;  
				CAN_ALL_ID[m].Port=port;
				CAN_ALL_ID[m].IDE=CAN_ID_EXT;
				// 打印ID
				sprintf(buf,"E 0x%08X",RxMsg->ExtId);
				USART_STR(USART1,buf);
				// 打印count
				sprintf(buf," -- %d - %03d\r\n",port,count);  
				USART_STR(USART1,buf);
				break;
			}
		}	
	}
}

/*****************************

  串口发送所有ID 字符串
*****************************/
void Comm_Send_All_ID_str(void)
{
    u8 m=0;	
	char buf[20];
	u8 Count=0;

	sprintf(buf,"All ID\r\n");  
	USART_STR(USART1,buf);

    for(m=0;m<CAN_TBUFFLEN;m++)
	{
	    if(CAN_ALL_ID[m].ID!=0xFFFFFFFF)  // 有记录
		{
		    Count++;
		    // 打印ID
			if(CAN_ALL_ID[m].IDE==CAN_ID_STD)
			    sprintf(buf,"S 0x%08X",CAN_ALL_ID[m].ID);
		    else
				sprintf(buf,"E 0x%08X",CAN_ALL_ID[m].ID);
			USART_STR(USART1,buf);

			// 打印count
			sprintf(buf," -- %d - %03d\r\n",CAN_ALL_ID[m].Port,Count);  
			USART_STR(USART1,buf);
		}
		else  // 查找完
		{
		    sprintf(buf,"Total %03d ID\r\n",Count);  
			USART_STR(USART1,buf);
			break;
		}
	}

}

/*****************************

  串口发送所有禁止转发ID  字符串

*****************************/
void Comm_Send_N_CrossTransmit_str(void)
{
    u8 m=0;	
	char buf[20];
	u8 Num=0;

	sprintf(buf,"All Not CrossTransmit ID\r\n");  
	USART_STR(USART1,buf);

    for(m=0;m<CAN_CBUFFLEN;m++)
	{
	    if(CAN_CrossSendCtrl[m]==0xFFFFFFFF)  // 空记录，不发送
		{
		    
		}
		else  // 有记录，发送
		{
			Num++;
		    // 打印ID
			sprintf(buf,"0x%08X",CAN_CrossSendCtrl[m]);
			USART_STR(USART1,buf);

			// 打印count
			sprintf(buf," -- %03d\r\n",Num);  
			USART_STR(USART1,buf);

		    
		}
	}

	sprintf(buf,"Total %03d ID\r\n",Num);  
	USART_STR(USART1,buf);

}

/*****************************

  设置或取消禁止转发ID
  ID：输入ID，存在则取消，不存在则设置
*****************************/
void Set_N_CrossTransmit(uint32_t ID)
{
    u8 m=0;	
	u8 end=1;
	char buf[50];

	// 查找是否有记录，有则清
    for(m=0;m<CAN_CBUFFLEN;m++)
	{
	    if(CAN_CrossSendCtrl[m]==ID)  // 已有记录
		{
		    CAN_CrossSendCtrl[m]=0xFFFFFFFF;  // 清
			sprintf(buf,"Clear Not CrossTransmit ID:0x%08X\r\n",ID);  
			USART_STR(USART1,buf);
			end=0;
			break;		       // 在中断中，不可用return
		}

	}

	if(end)
	{
		// 未有记录，查找空位设置
	    for(m=0;m<CAN_CBUFFLEN;m++)
		{
		    // 查找空位
		    if(CAN_CrossSendCtrl[m]==0xFFFFFFFF)  
			{
			    CAN_CrossSendCtrl[m]=ID;  // 设置
				sprintf(buf,"Set Not CrossTransmit ID:0x%08X\r\n",ID);  
				USART_STR(USART1,buf);
				end=0;
				break;		   // 在中断中，不可用return 
			}
		}	
	}

	if(end)
	{
		// 没有空位，未能设置
		sprintf(buf,"Buffer over,not Set\r\n");  
		USART_STR(USART1,buf);	
	}
	

}

/*****************************
  清空全部禁止转发ID
*****************************/
void Clear_All_N_CrossTransmit(void)
{
    u8 m=0;	
	char buf[50];

    for(m=0;m<CAN_CBUFFLEN;m++)
	{
		CAN_CrossSendCtrl[m]=0xFFFFFFFF;  // 清
	}

	sprintf(buf,"All Not CrossTransmit Clear\r\n");  
	USART_STR(USART1,buf);
}

/*****************************
  全部ID都设置成禁止转发
*****************************/
void SetAllID_N_CrossTansmit(void)
{
    u8 m=0;
	char buf[50];

	for(m=0;(m<CAN_TBUFFLEN)&&(m<CAN_CBUFFLEN);m++)
	{
	    CAN_CrossSendCtrl[m]=CAN_ALL_ID[m].ID;
	}

	sprintf(buf,"All ID Set to Not CrossTransmit\r\n");  
	USART_STR(USART1,buf);
}

/*****************************

  CAN转发一条数据 CAN1->CAN2 CAN2->CAN1
  port    数据来源CAN端口 1：CAN1  2：CAN2
  Msg	  CAN数据
*****************************/
void CAN_CrossTransmit(uint8_t port,CanRxMsg *Msg)
{
    CanTxMsg TXMsg={0,0,CAN_ID_STD,CAN_RTR_DATA,8,0,0,0,0,0,0,0,0};

    TXMsg.StdId=Msg->StdId;
	TXMsg.ExtId=Msg->ExtId;
	TXMsg.IDE=Msg->IDE;
	TXMsg.RTR=Msg->RTR;
	TXMsg.DLC=Msg->DLC;
	memcpy(TXMsg.Data,Msg->Data,8);  // Data
    if(port==0x01)	 // CAN1 -> CAN2
	{
	    CAN_SendData(CAN2,&TXMsg);
	}
	else
	if(port==0x02)  // CAN2 -> CAN1
	{
	    CAN_SendData(CAN1,&TXMsg);
	}
}

/*****************************
  检查是否在禁止转发记录中
  RETURN:  0  不在，可以转发
           1  在，禁止转发
*****************************/
uint8_t CheckNotCross(uint32_t ID)
{
    uint16_t x;
	for(x=0;x<CAN_CBUFFLEN;x++)
	{
	    if(ID==CAN_CrossSendCtrl[x])
		    return 1;
	}

	return 0;
}

/*****************************
  SHOW 常态变化的ID
*****************************/
void show_change_ID(void)
{
    uint8_t r,e,q;
	char Buf[60];

	// 清空显示区的缓冲
    for(r=0;r<CAN_TBUFFLEN;r++)
	{
	    if(change_toshow[r].ID!=0xFFFFFFFF) // 有数据
		    change_toshow[r].ID=0xFFFFFFFF;
		else
		    break;
	}
	// 从计算区复制到显示区
	e=0;
	for(r=0;r<CAN_TBUFFLEN;r++)
	{
	    if(change_tocount[r].ID!=0xFFFFFFFF) // 计算区有数据
		{
		    if(change_tocount[r].Tick>0)  // 次数>0 copy 
			{
			    memcpy(&change_toshow[e++],&change_tocount[r],sizeof(CAN_Data));			
			}


			memset(&change_tocount[r],0,sizeof(CAN_Data));  // 清空计算区
			change_tocount[r].ID=0xFFFFFFFF;
		}
		else
		    break;   
	}

	// show
	sprintf(Buf,"Change ID\r\n");    
	USART_STR(USART1,Buf);

	q=0;
	for(r=0;r<CAN_TBUFFLEN;r++)
	{
	    if(change_toshow[r].ID!=0xFFFFFFFF) // 显示区有数据
		{
		    // Port
			sprintf(Buf,"%d",change_toshow[r].Port);    
			USART_STR(USART1,Buf);
			// ID
			if(change_toshow[r].IDE==CAN_ID_STD)
			{
			    sprintf(Buf,",S0x%08X",change_toshow[r].ID);
		        USART_STR(USART1,Buf);
			}
			else
			{
			    sprintf(Buf,",E0x%08X",change_toshow[r].ID);
		        USART_STR(USART1,Buf);
			}
			// Ticks
			sprintf(Buf,"  %04d\r\n",change_toshow[r].Tick);
		    USART_STR(USART1,Buf);
			
			q++;		
		}
		else
		{
		    break;
		}
	}
	
	sprintf(Buf,"Tatal %d change\r\n",q);    
	USART_STR(USART1,Buf);
}

/*****************************
  SHOW 常态变化外的数据变化次数
*****************************/
void show_change_ticks(void)
{
    uint8_t m,n,flag=1,q=0;
	char Buf[60];

	sprintf(Buf,"Change Ticks\r\n");    
	USART_STR(USART1,Buf);

	// 检查计算区的数据
	for(m=0;m<CAN_TBUFFLEN;m++)
	{
	    if(change_tocount[m].ID!=0xFFFFFFFF) // 有数据
		{
		    if(change_tocount[m].Tick>0)  // 次数>0
			{
			    // 检查是否在常态变化区
			    for(n=0;n<CAN_TBUFFLEN;n++)
				{
				    if(change_toshow[n].ID!=0xFFFFFFFF)  
					{
					    if(change_toshow[n].ID==change_tocount[m].ID)  
						{
						    flag=0;
							break;	//在常态变化ID中
						}		    
					}
					else
					{
					    break; // 检查完常态变化区
					}
				}

				// 判断发送
				if(flag)
				{
				    // Port
					sprintf(Buf,"%d",change_tocount[m].Port);    
					USART_STR(USART1,Buf);
					// ID
					if(change_tocount[m].IDE==CAN_ID_STD)
					{
					    sprintf(Buf,",S0x%08X",change_tocount[m].ID);
				        USART_STR(USART1,Buf);
					}
					else
					{
					    sprintf(Buf,",E0x%08X",change_tocount[m].ID);
				        USART_STR(USART1,Buf);
					}
					// Ticks
					sprintf(Buf,"  %04d\r\n",change_tocount[m].Tick);
				    USART_STR(USART1,Buf);
					
					q++;				
				}

			}
			else
			{
			    continue;  // 次数不>0，下一条
			}
			    
		}
		else
		{
		    break; // 计算区完，无数据
		}
		    
	}

	sprintf(Buf,"Tatal %d change\r\n",q);    
	USART_STR(USART1,Buf);

}

/*****************************
  SHOW 常态变化外的数据
*****************************/
void show_not_change(void)
{
    uint8_t m,n,flag=1,q=0;
	char Buf[60];

	sprintf(Buf,"not Change\r\n");    
	USART_STR(USART1,Buf);

	// 检查计算区的数据
	for(m=0;m<CAN_TBUFFLEN;m++)
	{
	    if(change_tocount[m].ID!=0xFFFFFFFF) // 有数据
		{
			    // 检查是否在常态变化区
			    for(n=0;n<CAN_TBUFFLEN;n++)
				{
				    if(change_toshow[n].ID!=0xFFFFFFFF)  
					{
					    if(change_toshow[n].ID==change_tocount[m].ID)  
						{
						    flag=0;
							break;	//在常态变化ID中
						}		    
					}
					else
					{
					    break; // 检查完常态变化区
					}
				}

				// 判断发送
				if(flag)
				{
				    // Port
					sprintf(Buf,"%d",change_tocount[m].Port);    
					USART_STR(USART1,Buf);
					// ID
					if(change_tocount[m].IDE==CAN_ID_STD)
					{
					    sprintf(Buf,",S0x%08X",change_tocount[m].ID);
				        USART_STR(USART1,Buf);
					}
					else
					{
					    sprintf(Buf,",E0x%08X",change_tocount[m].ID);
				        USART_STR(USART1,Buf);
					}
					// Ticks
					sprintf(Buf,"  %04d\r\n",change_tocount[m].Tick);
				    USART_STR(USART1,Buf);
					
					q++;				
				}
			    
		}
		else
		{
		    break; // 计算区完，无数据
		}
		    
	}

	sprintf(Buf,"Tatal %d not change\r\n",q);    
	USART_STR(USART1,Buf);

}

/*****************************
  SHOW_CMD
*****************************/
void Debug_tick(void)
{
    switch(SHOW_CMD)
	{
	    case 0x01:	// shhow所有ID
		  SHOW_CMD=0;
		  Comm_Send_All_ID_str();
		  break;
		case 0x02:	// 所有禁止转发的ID
		  SHOW_CMD=0;
		  Comm_Send_N_CrossTransmit_str();
		  break;
		case 0x03:	// 设置或取消设置禁止转发ID
		  SHOW_CMD=0;
		  Set_N_CrossTransmit(Set_ID);
		  break;
		case 0x04:	// 清空全部禁止转发ID
		  SHOW_CMD=0;
		  Clear_All_N_CrossTransmit();
		  break;
		case 0x05: // 全部ID设置成禁止转发
		  SHOW_CMD=0;
		  SetAllID_N_CrossTansmit();
		  break;
		case 0x06:  // show常变ID
		  SHOW_CMD=0;
		  show_change_ID();
		  break;
		case 0x07:  // show常变ID外的变化ID次数
		  SHOW_CMD=0;
		  show_change_ticks();
		  break;
		case 0x08:  // show常态变化ID外的数据ID
		  SHOW_CMD=0;
		  show_not_change();
		default:
		  SHOW_CMD=0;
		  break;
	}    
}

/*****************************
  ID MASK控制打印
*****************************/
void Msg_Ctrl_Show(uint8_t port,CanRxMsg *RxMsg,uint16_t Tim)
{
    uint32_t IDtemp;
	if(RxMsg->IDE==CAN_ID_STD) // 标准帧
	    IDtemp=RxMsg->StdId;
	else
	    IDtemp=RxMsg->ExtId;

    if((ANA_PORT==1)||(ANA_PORT==2)) // 优先处理按端口打印
	{
	    if(ANA_PORT==port)
		    Comm_Send_CANmsg_str(port,RxMsg,Tim);
	}
	else  // 处理按ID打印
    if( (ANA_ID&ANA_MASK) == (ANA_MASK & IDtemp) )
	    Comm_Send_CANmsg_str(port,RxMsg,Tim);

    
}

uint8_t Send_EN=0;
CanTxMsg TXMsg={0,0,CAN_ID_STD,CAN_RTR_DATA,8,0,0,0,0,0,0,0,0};
CAN_TypeDef* Port=CAN1;

void proc_01(void)
{	
    if(Send_EN)
        CAN_SendData(Port,&TXMsg);
}

/*****************************
  中断1ms调用一次
*****************************/
void time_tick_1ms(void)
{
    static uint32_t tick_count=0;

	tick_count++;
	if(tick_count>=20)
	{
	    tick_count=0;
		proc_01();
	}

}







