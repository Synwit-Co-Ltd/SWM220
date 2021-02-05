#include "SWM220.h"


void SerialInit(void);

int main(void)
{	
	uint32_t i;
	
	SystemInit();
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0, 0);				//�� LED��ָʾ����ִ��״̬
	GPIO_SetBit(GPIOA, PIN5);						//����LED
	for(i = 0; i < SystemCoreClock/2; i++);			//��ʱ����ֹ�ϵ��SWD�����е��޷����س���
	
	GPIO_Init(GPIOA, PIN4, 0, 1, 0, 0);				//�Ӱ���������ʹ��
	SYS->PAWKEN |= (1 << PIN4);						//����PA4���ŵ͵�ƽ����
	
	while(1==1)
	{
		GPIO_SetBit(GPIOA, PIN5);					//����LED
		for(i = 0; i < SystemCoreClock/8; i++);
		GPIO_ClrBit(GPIOA, PIN5);					//Ϩ��LED
		
		EnterSleepMode();
		
		while((SYS->PAWKSR & (1 << PIN4)) == 0);	//�ȴ���������
		SYS->PAWKSR |= (1 << PIN4);					//�������״̬
	}
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTB, PIN11, PORTB_PIN11_UART0_RX, 1);	//GPIOB.11����ΪUART0��������
	PORT_Init(PORTB, PIN12, PORTB_PIN12_UART0_TX, 0);	//GPIOB.12����ΪUART0�������
 	
 	UART_initStruct.Baudrate = 57600;
	UART_initStruct.DataBits = UART_DATABIT_8;
	UART_initStruct.Parity = UART_PARITY_NONE;
	UART_initStruct.StopBits = UART_STOPBIT_1;
	UART_initStruct.RXThresholdIEn = 0;
	UART_initStruct.TXThresholdIEn = 0;
	UART_initStruct.TimeoutIEn = 0;
 	UART_Init(UART0, &UART_initStruct);
	UART_Open(UART0);
}

/****************************************************************************************************************************************** 
* ��������: fputc()
* ����˵��: printf()ʹ�ô˺������ʵ�ʵĴ��ڴ�ӡ����
* ��    ��: int ch		Ҫ��ӡ���ַ�
*			FILE *f		�ļ����
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}