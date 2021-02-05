#include "SWM220.h"

void SerialInit(void);

int main(void)
{
	ADC_InitStructure ADC_initStruct;
	
	SystemInit();
	
	SerialInit();
	
	PORT_Init(PORTA, PIN14, PORTA_PIN14_ADC_IN0, 0);//PA.14 => ADC.CH0
	PORT_Init(PORTA, PIN13, PORTA_PIN13_ADC_IN1, 0);//PA.13 => ADC.CH1
	PORT_Init(PORTA, PIN12, PORTA_PIN12_ADC_IN2, 0);//PA.12 => ADC.CH2
	PORT_Init(PORTA, PIN11, PORTA_PIN11_ADC_IN3, 0);//PA.11 => ADC.CH3
	PORT_Init(PORTA, PIN10, PORTA_PIN10_ADC_IN4, 0);//PA.10 => ADC.CH4
	PORT_Init(PORTA, PIN9,  PORTA_PIN9_ADC_IN5,  0);//PA.9  => ADC.CH5
	PORT_Init(PORTA, PIN8,  PORTA_PIN8_ADC_IN6,  0);//PA.8  => ADC.CH6
	PORT_Init(PORTA, PIN7,  PORTA_PIN7_ADC_IN7,  0);//PA.7  => ADC.CH7
	
	ADC_initStruct.clk_src = ADC_CLKSRC_SYSCLK_DIV4;
	ADC_initStruct.clk_div = 24;
	ADC_initStruct.channels = ADC_CH0 | ADC_CH1 | ADC_CH2 | ADC_CH3 | ADC_CH4 | ADC_CH5 | ADC_CH6 | ADC_CH7;
	ADC_initStruct.trig_src = ADC_TRIGSRC_SW;
	ADC_initStruct.samplAvg = ADC_AVG_SAMPLE1;
	ADC_initStruct.Continue = 0;					//������ģʽ��������ģʽ
	ADC_initStruct.EOC_IEn = ADC_CH7;				//�������һ����ת����ͨ���������ͨ�����ʱ8��ͨ���Ͷ�ת�������
	ADC_initStruct.OVF_IEn = 0;
	ADC_Init(ADC, &ADC_initStruct);					//����ADC
			
	ADC_Open(ADC);									//ʹ��ADC
	ADC_Start(ADC);									//����ADC����ʼת��
	
	while(1==1)
	{
	}
}

void ADC_Handler(void)
{	
	printf("%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\t%4d\r\n", ADC_Read(ADC, ADC_CH0), ADC_Read(ADC, ADC_CH1), ADC_Read(ADC, ADC_CH2), ADC_Read(ADC, ADC_CH3),
												         ADC_Read(ADC, ADC_CH4), ADC_Read(ADC, ADC_CH5), ADC_Read(ADC, ADC_CH6), ADC_Read(ADC, ADC_CH7));
	
	ADC_IntEOCClr(ADC, ADC_CH7);		//����жϱ�־

	ADC_Start(ADC);						//�ٴ����������ϵõ��������ӡ
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