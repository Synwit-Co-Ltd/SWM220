#include "SWM220.h"


void SerialInit(void);

/* ע�⣺�˳���ֻ���ڵ�Ƭ���ٶ�С��25MHzʱִ��
*/
int main(void)
{
	uint32_t cnt1, cnt2;
	volatile uint32_t x = 123456789 ;
	volatile uint32_t y = 765;
	volatile uint32_t z;
	
	SystemInit();
	
	SerialInit();
	
	SysTick_Config(1000000);
	z = x / y;
	cnt1 = 1000000 - SysTick->VAL;
	SysTick->CTRL = 0;
	printf("Before Flash Speed X2: %d clock\r\n", cnt1);
	
	FlashSpeedX2();
	
	SysTick_Config(1000000);
	z = x / y;
	cnt2 = 1000000 - SysTick->VAL;
	SysTick->CTRL = 0;
	printf("After Flash Speed X2: %d clock\r\n", cnt2);
	
	printf("program execute speed promote %d%%\r\n", (cnt1 - cnt2) * 100 / cnt2);
	
	while(1==1)
	{
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
