#include "GPS_UART.h"
#include "USART.h"

#define GPS_UART_BUF_SIZE 255
u8 GPS_UART_TX_BUF[GPS_UART_BUF_SIZE];
u8 GPS_UART_RX_BUF[GPS_UART_BUF_SIZE];

Queue_ GPS_UART_QueueSend;
Queue_ GPS_UART_QueueRecv;

void GPS_UART_init(u32 bound);
BOOL GPS_send(u8 *data, u16 num);
BOOL GPS_receive(u8 *data, u16 num);
void GPS_Cof(void);
struct GPS_UART_ GPS_UART= {  
	GPS_UART_init, 
	GPS_send,	
	GPS_receive,
	GPS_Cof
};


void GPS_UART_init(u32 Bound)
{
	//GPIO
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
	
	//GPS_TX   PA.0
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPS_RX	  PA.1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPS NVIC 
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	

	USART_InitStructure.USART_BaudRate = Bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(UART4, &USART_InitStructure);
 
	GPS_UART_QueueSend = Queue_OPS.Init(GPS_UART_BUF_SIZE,GPS_UART_TX_BUF);
	GPS_UART_QueueRecv = Queue_OPS.Init(GPS_UART_BUF_SIZE,GPS_UART_RX_BUF);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);      
	USART_Cmd(UART4, ENABLE);
}
/**************************************************************
	串口4中断
**************************************************************/
extern "C"{
void UART4_IRQHandler(void)                	//串口1中断服务程序
{
	u8 temp;
  //send 
  if(USART_GetITStatus(UART4, USART_IT_TXE) != RESET){   
		if(True == Queue_OPS.Dequeue(&temp,&GPS_UART_QueueSend))
			USART_SendData(UART4, temp); 
		else
			USART_ITConfig(UART4, USART_IT_TXE, DISABLE);   
  }
	
		//receive 缓冲满不接受数据
  if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET){    
		
		temp = USART_ReceiveData(UART4);
    USART_ClearITPendingBit(UART4, USART_IT_RXNE);
		
		if(False == Queue_OPS.Enqueue(temp,&GPS_UART_QueueRecv))
		{}
  }
} 
}

/**************************************************************
	串口4发送
**************************************************************/
BOOL GPS_send(u8 *data, u16 num)
{
	u16 i = 0;
	if(num > GPS_UART_QueueSend.Size - GPS_UART_QueueSend.Length)
		return False;
	for(i = 0;i < num;i++){
		if(False == Queue_OPS.Enqueue(data[i],&GPS_UART_QueueSend)){
			return False;
		}		
	}
	USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
	return  True;
}
/**************************************************************
	串口4接收
**************************************************************/
BOOL GPS_receive(u8 *data, u16 num)
{
	u16 i = 0;
	if(num > GPS_UART_QueueRecv.Length)
		return False;
	for(i = 0;i < num;i++){
		if(False == Queue_OPS.Dequeue(&data[i],&GPS_UART_QueueRecv)){
			return False;
		}
	}
	return True;	
}


void GPS_Cof(void)
{
	#if USE_GPS_PC
	extern Queue_ USART1_QueueRecv;
	u8 gps_2_pc[100],pc_2_gps[100];
	u16 gps_2_pc_size = 0;
	u16 pc_2_gps_size = 0;
	
	if(GPS_UART_QueueRecv.Length != 0)
	{
		gps_2_pc_size = GPS_UART_QueueRecv.Length;
		GPS_receive(gps_2_pc,gps_2_pc_size);
		usart1.send(gps_2_pc,gps_2_pc_size);
	}
	
	if(USART1_QueueRecv.Length != 0)
	{
		pc_2_gps_size = USART1_QueueRecv.Length;
		usart1.receive(pc_2_gps,USART1_QueueRecv.Length);	
		GPS_send(pc_2_gps,pc_2_gps_size);	
	}
	#endif
}


