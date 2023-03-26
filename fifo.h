/*
*********************************************************************************************************
*
*	模块名称 : 串口中断+FIFO驱动模块
*	文件名称 : bsp_uart_fifo.h
*	版    本 : V1.0
*	说    明 : 头文件
*
*	Copyright (C), 2013-2014, 传人记
*
*********************************************************************************************************
*/
#include "plc_conf.h"

#ifndef _BSP_USART_FIFO_H_
#define _BSP_USART_FIFO_H_

/*
	如果需要更改串口对应的管脚，请自行修改 bsp_uart_fifo.c文件中的 static void InitHardUart(void)函数
*/

/* 定义使能的串口, 0 表示不使能（不增加代码大小）， 1表示使能 */
/*
	传人记STM32-V4 串口分配：
	【串口1】 RS232 芯片第1路。
		PA9/USART1_TX	  --- 打印调试口
		PA10/USART1_RX

	【串口2】GPRS 模块 
		PA2/USART2_TX
		PA3/USART2_RX 

	【串口3】 RS485 通信 - TTL 跳线 和 排针
		PB10/USART3_TX
		PB11/USART3_RX
		PB2/BOOT1/HS_MOD1_RS485_TX_EN

	【串口4】 --- 不做串口用。
	【串口5】 --- 不做串口用。
*/
#define	UART1_FIFO_EN	0
#define	UART2_FIFO_EN	0
#define	UART3_FIFO_EN	1
#define	UART4_FIFO_EN	1
#define	UART5_FIFO_EN	1

/* 定义端口号 */
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 */
	COM2 = 1,	/* USART2, PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4,	/* UART5, PC12, PD2 */
}COM_PORT_E;

/* 定义串口波特率和FIFO缓冲区大小，分为发送缓冲区和接收缓冲区, 支持全双工 */
#if UART1_FIFO_EN == 1
	#define UART1_BAUD			  115200
	#define UART1_TX_BUF_SIZE	1024
	#define UART1_RX_BUF_SIZE	1024
#endif

#if UART2_FIFO_EN == 1
//	#define UART2_BAUD			  115200
  #define UART2_BAUD			  9600
	#define UART2_TX_BUF_SIZE	512
	#define UART2_RX_BUF_SIZE	512
#endif

#if UART3_FIFO_EN == 1
//	#define UART3_BAUD			  115200
	#define UART3_BAUD			  9600
	#define UART3_TX_BUF_SIZE	1024
	#define UART3_RX_BUF_SIZE	1024
#endif

#if UART4_FIFO_EN == 1
	//#define UART4_BAUD			115200
	#define UART4_BAUD			  9600
	#define UART4_TX_BUF_SIZE	1024
	#define UART4_RX_BUF_SIZE	1024
#endif

#if UART5_FIFO_EN == 1
	//#define UART5_BAUD			115200
	#define UART5_BAUD			  9600
	#define UART5_TX_BUF_SIZE	1024
	#define UART5_RX_BUF_SIZE	1024
#endif

/* 串口设备结构体 */
typedef struct
{
	USART_TypeDef *uart;				// STM32内部串口设备指针
	uint8_t com;                // 端口号
	uint32_t baud;              // 波特率	
	
	uint8_t *pTxBuf;						// 发送缓冲区 
	uint8_t *pRxBuf;						// 接收缓冲区 
	uint16_t txBufSize;					// 发送缓冲区大小 
	uint16_t rxBufSize;					// 接收缓冲区大小 
	__IO uint16_t txWrite;			// 发送缓冲区写指针 
	__IO uint16_t txRead;				// 发送缓冲区读指针 
	__IO uint16_t txCnt;			// 等待发送的数据个数 

	__IO uint16_t rxWrite;		// 接收缓冲区写指针 
	__IO uint16_t rxRead;			// 接收缓冲区读指针 
	__IO uint16_t rxCnt;		// 还未读取的新数据个数 

	void (*SendBefor)(uint8_t);   	// 开始发送之前的回调函数指针（主要用于RS485切换到发送模式） 
	void (*SendOver)(uint8_t); 	  // 发送完毕的回调函数指针（主要用于RS485将发送模式切换为接收模式） 
	void (*ReciveNew)(uint8_t _com,uint16_t _baud,uint8_t _data);	// 串口收到数据的回调函数指针 
}UART_T;

/* 传人记，20190618新增 */
typedef enum
{
    NONE =0,          
    ODD =1,            
    EVEN =3        
} eParity;

typedef enum
{
	Band600=4,
	Band1200,
	Band2400,
	Band4800,
	Band9600,
	Band19200,
	Band38400,
	Band56000,
	Band57600,
	Band115200
}eBand;

typedef enum
{
    RTU,                    
    ASCII,                   
    TCP                     
} eMode;

typedef union
{
  struct 
  {
    u8 dataBits    :1; // b0：数据长度，0为7位、1为8位
    eParity parity :2; // b1~2：校验
    u8 stopBits    :1; // b3：停止位
    eBand rate     :4; // b4~7：波特率
	  u8 stx         :1; // b8：报头，0：无；1:有[D8124初始值：STX(02H)]，仅RS用
		u8 etx         :1; // b9：报尾，0:无; 1:有[D8125初始值：ETX(03H)]，仅RS用
		u8 reserved0   :3; // b10~12：预留
    u8 sumCheck    :1; // b13: 0：不附加；1：附加，仅RS用
    u8 protocol    :1; // b14：0：专用协议(MODBUS)；1：无协议(RS)
    u8 hs          :1; // b15：0为从机；1为主机,仅MODBUS用
  } bits;                    
  u16 uartParm;                
}uUartParm;         

extern uUartParm lastUP[]; // 通信参数

void bsp_InitUart(void);
void comSendBuf(uint8_t _com, uint8_t *_ucaBuf, uint16_t _usLen);
void comSendChar(uint8_t _com, uint8_t _ucByte);
uint8_t comGetChar(uint8_t _com, uint8_t *_pByte);

void comClearTxFifo(uint8_t _com);
void comClearRxFifo(uint8_t _com);


void RS485_SendStr(uint8_t _com,char *_pBuf);
void RS485_SendBuf(uint8_t _com,uint8_t *_ucaBuf, uint16_t _usLen);

void RS485_SendBefor(uint8_t _com);
void RS485_SendOver(uint8_t _com);
void RS485_ReciveNew(uint8_t _com,uint16_t _baud,uint8_t _byte);

//void bsp_SetUartBaud(USART_TypeDef* USARTx,uint32_t _baud);
void bsp_SetUartBaud(USART_TypeDef* USARTx, uUartParm uup);
void UART_ParmSelect(void);
void MOD_HostSlaveModeSelect(void);
#endif

/***************************** 传人记 (END OF FILE) *********************************/

