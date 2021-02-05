#ifndef __SWM220_UART_H__
#define __SWM220_UART_H__

typedef struct {
	uint32_t Baudrate;
	
	uint8_t  DataBits;
	
	uint8_t  Parity;
	
	uint8_t  StopBits;
	
	uint8_t  RXThreshold;			//ȡֵ0--7
	uint8_t  RXThresholdIEn;		//��RX FIFO�����ݸ��� >  RXThresholdʱ�����ж�
	
	uint8_t  TXThreshold;			//ȡֵ0--7
	uint8_t  TXThresholdIEn;		//��TX FIFO�����ݸ��� <= TXThresholdʱ�����ж�
	
	uint8_t  TimeoutTime;			//��ʱʱ�� = TimeoutTime/(Baudrate/10) ��
	uint8_t  TimeoutIEn;			//��ʱ�жϣ�RX FIFO�ǿգ��ҳ��� TimeoutTime/(Baudrate/10) ��û����RX���Ͻ��յ�����ʱ�����ж�
} UART_InitStructure;


#define UART_PARITY_NONE	0
#define UART_PARITY_ODD		1
#define UART_PARITY_EVEN	3
#define UART_PARITY_1		5
#define UART_PARITY_0		7

#define UART_DATABIT_8		0
#define UART_DATABIT_9		1

#define UART_STOPBIT_1		0
#define UART_STOPBIT_2		1

#define UART_ERR_FRAME		1
#define UART_ERR_PARITY		2
#define UART_ERR_NOISE		3

void UART_Init(UART_TypeDef * UARTx, UART_InitStructure * initStruct);	//UART���ڳ�ʼ��
void UART_Open(UART_TypeDef * UARTx);
void UART_Close(UART_TypeDef * UARTx);

void UART_WriteByte(UART_TypeDef * UARTx, uint8_t data);				//����һ���ֽ�����
uint32_t UART_ReadByte(UART_TypeDef * UARTx, uint32_t * data);			//��ȡһ���ֽ����ݣ���ָ�������Ƿ�Valid

uint32_t UART_IsTXBusy(UART_TypeDef * UARTx);
uint32_t UART_IsRXFIFOEmpty(UART_TypeDef * UARTx);						//����FIFO�Ƿ�գ������������Լ���UART_ReadByte()
uint32_t UART_IsTXFIFOFull(UART_TypeDef * UARTx);						//����FIFO�Ƿ����������������Լ���UART_WriteByte()


void UART_SetBaudrate(UART_TypeDef * UARTx, uint32_t baudrate);			//���ò�����
uint32_t UART_GetBaudrate(UART_TypeDef * UARTx);			 			//��ȡ��ǰʹ�õĲ�����


void UART_INTRXThresholdEn(UART_TypeDef * UARTx);
void UART_INTRXThresholdDis(UART_TypeDef * UARTx);
uint32_t UART_INTRXThresholdStat(UART_TypeDef * UARTx);
void UART_INTTXThresholdEn(UART_TypeDef * UARTx);
void UART_INTTXThresholdDis(UART_TypeDef * UARTx);
uint32_t UART_INTTXThresholdStat(UART_TypeDef * UARTx);
void UART_INTTimeoutEn(UART_TypeDef * UARTx);
void UART_INTTimeoutDis(UART_TypeDef * UARTx);
uint32_t UART_INTTimeoutStat(UART_TypeDef * UARTx);


#endif //__SWM220_UART_H__
