#include "SWM220.h"

uint32_t TStart = 0xFFFFFF;	//Timer��ʼֵ
uint32_t Period = 0,		//PWM���ڳ���
	     LWidth = 0;		//PWM�͵�ƽ���

void SerialInit(void);
void TestSignal(void);

int main(void)
{	
	uint32_t i;
	
	SystemInit();
	
	SerialInit();
	
	TestSignal();	//���������źŹ�HALL���ܲ���
	
	PORT_Init(PORTA, PIN0, PORTA_PIN0_HALL_A, 1);	//PA0 -> HALL_A
	
	TIMR_Init(TIMR0, TIMR_MODE_TIMER, TStart, 1);
	
	TIMRG->HALLCR &= ~TIMRG_HALLCR_IEA_Msk;
	TIMRG->HALLCR |= (3 << TIMRG_HALLCR_IEA_Pos);	//HALL_A˫���ز����ж�
	
	TIMR_Start(TIMR0);
	
	while(1==1)
	{
		printf("Period = %d, HWidth = %d, Duty = %%%d\r\n", Period, Period-LWidth, (Period-LWidth)*100/Period);
		
		for(i = 0; i < 1000000; i++);
	}
}

void TIMR0_Handler(void)
{
	if(TIMRG->IF0 & TIMRG_IF_IF_Msk)
	{
		TIMRG->IF0 = (1 << TIMRG_IF_IF_Pos);			//��������жϣ�����жϱ�־
	}
	
	if(TIMRG->HALLSR & TIMRG_HALLSR_IFA_Msk)
	{
		if(TIMRG->HALLSR & TIMRG_HALLSR_STA_Msk)		//������
		{
			LWidth = TStart > TIMRG->HALL_A ? TStart - TIMRG->HALL_A : (0xFFFFFF + TStart) - TIMRG->HALL_A;
		}
		else											//�½���
		{
			Period = TStart > TIMRG->HALL_A ? TStart - TIMRG->HALL_A : (0xFFFFFF + TStart) - TIMRG->HALL_A;
			
			TStart = TIMRG->HALL_A;
		}
		
		TIMRG->HALLSR = (1 << TIMRG_HALLSR_IFA_Pos);	//����жϱ�־
	}
}

void TestSignal(void)
{
	PWM_InitStructure  PWM_initStruct;
	
	PORT_Init(PORTC, PIN6,  PORTC_PIN6_PWM1A,  0);
	PORT_Init(PORTB, PIN13, PORTB_PIN13_PWM1B, 0);
	
	PWM_initStruct.clk_div = PWM_CLKDIV_4;		//F_PWM = 24M/4 = 6MHz
	
	PWM_initStruct.mode = PWM_MODE_INDEP;		//A·��B·�������					
	PWM_initStruct.cycleA = 1000;				//6MHz/1000 = 6KHz
	PWM_initStruct.hdutyA =  250;				//250/1000 = 25%
	PWM_initStruct.initLevelA = 0;
	PWM_initStruct.cycleB = 1000;
	PWM_initStruct.hdutyB =  500;				//500/1000 = 50%
	PWM_initStruct.initLevelB = 1;
	PWM_initStruct.HEndAIEn = 0;
	PWM_initStruct.NCycleAIEn = 0;
	PWM_initStruct.HEndBIEn = 0;
	PWM_initStruct.NCycleBIEn = 0;
	
	PWM_Init(PWM1, &PWM_initStruct);
	PWM_Start(PWM1, 1, 1);
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
