#include "SWM220.h"

void SerialInit(void);

int main(void)
{
	ADC_InitStructure ADC_initStruct;
	PWM_InitStructure PWM_initStruct;
	
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
	ADC_initStruct.channels = ADC_CH1;
	ADC_initStruct.trig_src = ADC_TRIGSRC_PWM;
	ADC_initStruct.samplAvg = ADC_AVG_SAMPLE1;
	ADC_initStruct.Continue = 0;					//非连续模式，即单次模式
	ADC_initStruct.EOC_IEn = ADC_CH1;
	ADC_initStruct.OVF_IEn = 0;
	ADC_Init(ADC, &ADC_initStruct);					//配置ADC
	
	ADC_Open(ADC);									//使能ADC
	
	
	PORT_Init(PORTC, PIN6,  PORTC_PIN6_PWM1A, 0);
	PORT_Init(PORTB, PIN13, PORTB_PIN13_PWM1B, 0);
	
	PWM_initStruct.clk_div = PWM_CLKDIV_4;		//F_PWM = 24M/4 = 6M
	
	PWM_initStruct.mode = PWM_MODE_COMPL_CALIGN;//A路和B路独立输出					
	PWM_initStruct.cycleA = 10000;				//6M/10000 = 600Hz			
	PWM_initStruct.hdutyA =  2500;				//2500/10000 = 25%
	PWM_initStruct.initLevelA = 0;
	PWM_initStruct.cycleB = 10000;
	PWM_initStruct.hdutyB =  5000;				//5000/10000 = 50%
	PWM_initStruct.initLevelB = 1;
	PWM_initStruct.HEndAIEn = 0;
	PWM_initStruct.NCycleAIEn = 0;
	PWM_initStruct.HEndBIEn = 0;
	PWM_initStruct.NCycleBIEn = 0;
	
	PWM_Init(PWM1, &PWM_initStruct);
	
	PWMG->ADTRG1A = (1 << PWMG_ADTRG1A_EN_Pos) |
					(0 << PWMG_ADTRG1A_EVEN_Pos) |		//奇数周期生效
					(100 << PWMG_ADTRG1A_VALUE_Pos);
	
	PWMG->ADTRG1B = (1 << PWMG_ADTRG1B_EN_Pos) |
					(0 << PWMG_ADTRG1B_EVEN_Pos) |
					(100 << PWMG_ADTRG1B_VALUE_Pos);
	
	PWM_Start(PWM1, 1, 1);
	
	while(1==1)
	{
	}
}

void ADC_Handler(void)
{	
	printf("%d,", ADC_Read(ADC, ADC_CH1));
	
	ADC_IntEOCClr(ADC, ADC_CH1);	//清除中断标志
}


void SerialInit(void)
{
	UART_InitStructure UART_initStruct;
	
	PORT_Init(PORTB, PIN11, PORTB_PIN11_UART0_RX, 1);	//GPIOB.11配置为UART0输入引脚
	PORT_Init(PORTB, PIN12, PORTB_PIN12_UART0_TX, 0);	//GPIOB.12配置为UART0输出引脚
 	
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
* 函数名称: fputc()
* 功能说明: printf()使用此函数完成实际的串口打印动作
* 输    入: int ch		要打印的字符
*			FILE *f		文件句柄
* 输    出: 无
* 注意事项: 无
******************************************************************************************************************************************/
int fputc(int ch, FILE *f)
{
	UART_WriteByte(UART0, ch);
	
	while(UART_IsTXBusy(UART0));
 	
	return ch;
}
