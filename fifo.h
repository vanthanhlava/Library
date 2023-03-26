/*
*********************************************************************************************************
*
*	ģ������ : �����ж�+FIFO����ģ��
*	�ļ����� : bsp_uart_fifo.h
*	��    �� : V1.0
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2013-2014, ���˼�
*
*********************************************************************************************************
*/
#include "plc_conf.h"

#ifndef _BSP_USART_FIFO_H_
#define _BSP_USART_FIFO_H_

/*
	�����Ҫ���Ĵ��ڶ�Ӧ�Ĺܽţ��������޸� bsp_uart_fifo.c�ļ��е� static void InitHardUart(void)����
*/

/* ����ʹ�ܵĴ���, 0 ��ʾ��ʹ�ܣ������Ӵ����С���� 1��ʾʹ�� */
/*
	���˼�STM32-V4 ���ڷ��䣺
	������1�� RS232 оƬ��1·��
		PA9/USART1_TX	  --- ��ӡ���Կ�
		PA10/USART1_RX

	������2��GPRS ģ�� 
		PA2/USART2_TX
		PA3/USART2_RX 

	������3�� RS485 ͨ�� - TTL ���� �� ����
		PB10/USART3_TX
		PB11/USART3_RX
		PB2/BOOT1/HS_MOD1_RS485_TX_EN

	������4�� --- ���������á�
	������5�� --- ���������á�
*/
#define	UART1_FIFO_EN	0
#define	UART2_FIFO_EN	0
#define	UART3_FIFO_EN	1
#define	UART4_FIFO_EN	1
#define	UART5_FIFO_EN	1

/* ����˿ں� */
typedef enum
{
	COM1 = 0,	/* USART1  PA9, PA10 */
	COM2 = 1,	/* USART2, PA2, PA3 */
	COM3 = 2,	/* USART3, PB10, PB11 */
	COM4 = 3,	/* UART4, PC10, PC11 */
	COM5 = 4,	/* UART5, PC12, PD2 */
}COM_PORT_E;

/* ���崮�ڲ����ʺ�FIFO��������С����Ϊ���ͻ������ͽ��ջ�����, ֧��ȫ˫�� */
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

/* �����豸�ṹ�� */
typedef struct
{
	USART_TypeDef *uart;				// STM32�ڲ������豸ָ��
	uint8_t com;                // �˿ں�
	uint32_t baud;              // ������	
	
	uint8_t *pTxBuf;						// ���ͻ����� 
	uint8_t *pRxBuf;						// ���ջ����� 
	uint16_t txBufSize;					// ���ͻ�������С 
	uint16_t rxBufSize;					// ���ջ�������С 
	__IO uint16_t txWrite;			// ���ͻ�����дָ�� 
	__IO uint16_t txRead;				// ���ͻ�������ָ�� 
	__IO uint16_t txCnt;			// �ȴ����͵����ݸ��� 

	__IO uint16_t rxWrite;		// ���ջ�����дָ�� 
	__IO uint16_t rxRead;			// ���ջ�������ָ�� 
	__IO uint16_t rxCnt;		// ��δ��ȡ�������ݸ��� 

	void (*SendBefor)(uint8_t);   	// ��ʼ����֮ǰ�Ļص�����ָ�루��Ҫ����RS485�л�������ģʽ�� 
	void (*SendOver)(uint8_t); 	  // ������ϵĻص�����ָ�루��Ҫ����RS485������ģʽ�л�Ϊ����ģʽ�� 
	void (*ReciveNew)(uint8_t _com,uint16_t _baud,uint8_t _data);	// �����յ����ݵĻص�����ָ�� 
}UART_T;

/* ���˼ǣ�20190618���� */
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
    u8 dataBits    :1; // b0�����ݳ��ȣ�0Ϊ7λ��1Ϊ8λ
    eParity parity :2; // b1~2��У��
    u8 stopBits    :1; // b3��ֹͣλ
    eBand rate     :4; // b4~7��������
	  u8 stx         :1; // b8����ͷ��0���ޣ�1:��[D8124��ʼֵ��STX(02H)]����RS��
		u8 etx         :1; // b9����β��0:��; 1:��[D8125��ʼֵ��ETX(03H)]����RS��
		u8 reserved0   :3; // b10~12��Ԥ��
    u8 sumCheck    :1; // b13: 0�������ӣ�1�����ӣ���RS��
    u8 protocol    :1; // b14��0��ר��Э��(MODBUS)��1����Э��(RS)
    u8 hs          :1; // b15��0Ϊ�ӻ���1Ϊ����,��MODBUS��
  } bits;                    
  u16 uartParm;                
}uUartParm;         

extern uUartParm lastUP[]; // ͨ�Ų���

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

/***************************** ���˼� (END OF FILE) *********************************/

