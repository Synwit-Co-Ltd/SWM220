#include "SWM220.h"

void SerialInit(void);

int main(void)
{	
	uint32_t i;
	uint32_t rxdata, txdata = 0x23;
	SPI_InitStructure SPI_initStruct;
	
	SystemInit();
	
	SerialInit();	
	
	PORT_Init(PORTB, PIN13, PORTB_PIN13_SPI0_SSEL, 0);
	PORT_Init(PORTC, PIN1,  PORTC_PIN1_SPI0_MISO,  1);
	PORT_Init(PORTC, PIN2,  PORTC_PIN2_SPI0_MOSI,  0);
	PORT_Init(PORTC, PIN0,  PORTC_PIN0_SPI0_SCLK,  0);
	
	SPI_initStruct.clkDiv = SPI_CLKDIV_16;
	SPI_initStruct.FrameFormat = SPI_FORMAT_SPI;
	SPI_initStruct.SampleEdge = SPI_SECOND_EDGE;
	SPI_initStruct.IdleLevel = SPI_HIGH_LEVEL;
	SPI_initStruct.FrameSize = 8;
	SPI_initStruct.Master = 1;
	SPI_initStruct.RXFullIEn = 0;
	SPI_initStruct.RXHFullIEn = 0;
	SPI_initStruct.TXEmptyIEn = 0;
	SPI_initStruct.TXHFullIEn = 0;
	SPI_Init(SPI0, &SPI_initStruct);
	
	SPI_Open(SPI0);
	
	while(1==1)
	{
		rxdata = SPI_ReadWrite(SPI0, txdata);
		txdata = rxdata + 1;
		
		printf("rxdata: %d\r\n", rxdata);
		
		for(i = 0; i < SystemCoreClock/16; i++);
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