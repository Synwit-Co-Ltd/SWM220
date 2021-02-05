2018/5/30
CSL\SWM220_StdPeriph_Driver\SWM220_gpio.c�ļ������ȷ�����š���-��-д������ԭ���Ե�GPIO_Atomic���ͺ���

2018/6/13
CSL\CMSIS\DeviceSupport\SWM220.h�ļ��У�RTC_TypeDef���У׼�üĴ���CALIBREFCNT��CALIBEN��CALIBST����
CSL\SWM220_StdPeriph_Driver\SWM220_rtc.c�ļ��У�RTC_AlarmSetup()���������䣺if(alarmStruct->AlarmIEn) NVIC_EnableIRQ(RTC_IRQn);
���RTCУ׼��ʾ����RTC\SimplRTC_Calibrate

2018/6/27
CSL\SWM220_StdPeriph_Driver\SWM220_can.h�ļ���struct CAN_RXMessage�Ķ��������format�ֶΣ��������յ��������Ϣ�Ǳ�׼֡������չ֡
CSL\SWM220_StdPeriph_Driver\SWM220_can.c�ļ���CAN_Receive()�����и�CAN_RXMessage->format��ȷ��ֵ
CAN\SimplCAN_RX_Interrupt�����д�ӡ��Ϣʱ��ӡ��CAN_RXMessage->format�ֶ�

2018/8/17
CSL\SWM220_StdPeriph_Driver\SWM220_uart.c�ļ���UART_SetBaudrate()�����У�����
UARTx->BAUD |= ((SystemCoreClock/16/baudrate) << UART_BAUD_BAUD_Pos);
����Ϊ��
UARTx->BAUD |= ((SystemCoreClock/16/baudrate - 1) << UART_BAUD_BAUD_Pos);

2018/9/25
���CSL\SWM220_StdPeriph_Driver\SWM220_sleep.c�ļ�
CSL\SWM220_StdPeriph_Driver\SWM220_flash.c�ļ���AppToIsp()������__dsb(0)�滻��__DSB()
CSL\CMSIS\DeviceSupport\SWM220.h�ļ���struct SYS_TypeDef�����У���RESETST������ΪRSTST�����RTCWK��λ����

2018/9/26
CSL\CMSIS\DeviceSupport\SWM220.h�ļ���struct RTC_TypeDef������LOAD�Ĵ������λ����
CSL\SWM220_StdPeriph_Driver\SWM220_rtc.c�ļ���RTC_Init()��RTC_AlarmSetup()��RTC->LOAD�Ĵ�����������

2018/10/29
CSL\SWM220_StdPeriph_Driver\SWM220_can.c�ļ���CAN_SetFilter32b()��CAN_SetFilter16b()�����У�����
CANx->FILTER.AMR[0] = initStruct->FilterMask32b & 0xFF;
CANx->FILTER.AMR[1] = (initStruct->FilterMask32b >>  8) & 0xFF;
... ...
����Ϊ��
CANx->FILTER.AMR[3] = initStruct->FilterMask32b & 0xFF;
CANx->FILTER.AMR[2] = (initStruct->FilterMask32b >>  8) & 0xFF;

2019/01/16
CSL\SWM220_StdPeriph_Driver\SWM220_dma.c�ļ��� DMA_CH_Config()�����У�����
DMA->IF  = (1 << chn);		//����жϱ�־
DMA->IE |= (1 << chn);
if(int_en)  DMA->IM &= ~(1 << chn);
else        DMA->IM |=  (1 << chn);
����Ϊ��
DMA->IE = 0x35;			//��ʹ��ʹ���жϣ�Ҳ���Բ�ѯ״̬/��־
DMA_CH_INTClr(chn);		//����жϱ�־
if(int_en) DMA_CH_INTEn(chn);
else	   DMA_CH_INTDis(chn);

2019/01/17
CSL\CMSIS\DeviceSupport\SWM220.h�ļ��У�DMA->CR.WENλλ�ø���

2019/03/04
CSL\SWM220_StdPeriph_Driver\SWM220_uart.c �ļ��� UART_GetBaudrate() �н���
return (UARTx->BAUD & UART_BAUD_BAUD_Msk);
����Ϊ��
return SystemCoreClock/16/(((UARTx->BAUD & UART_BAUD_BAUD_Msk) >> UART_BAUD_BAUD_Pos) + 1);

CSL\SWM220_StdPeriph_Driver\SWM220_sleep.c �ļ��У��� EnterStopMode1() ����Ϊ��EnterStopMode()

2019/03/07
CSL\SWM220_StdPeriph_Driver\SWM220_adc.c �ļ��� ADC_Init() �н���
if(initStruct->clk_src == ADC_CLKSRC_XTAL_DIV4)
{
	PORT_Init(PORTD, PIN3, PORTD_PIN3_XTAL_IN,  1);
	PORT_Init(PORTD, PIN2, PORTD_PIN2_XTAL_OUT, 1);
	
	SYS->XTALCR = (1 << SYS_XTALCR_EN_Pos);
}
ɾ������ADCʹ�þ�����Ϊʱ��Դ����������δ�������������û��ڳ�ʼ��ADC֮ǰ����֮

2019/04/18
CSL\CMSIS\DeviceSupport\SWM220.h �ļ��У�����CAN->ECC�Ĵ���λ����

2019/10/15
CSL\CMSIS\DeviceSupport\SWM220.h �ļ��У�UART->BAUD�Ĵ����е� RXD λ������Ϊ RXDn

2019/11/07
CSL\SWM220_StdPeriph_Driver\SWM220_spi.c�ļ������SPI_INTRXHalfFullClr()��SPI_INTRXFullClr()��SPI_INTTXHalfFullClr()��SPI_INTTXEmptyClr()�ĸ��жϱ�־�������

2020/03/03
CSL\SWM220_StdPeriph_Driver\SWM220_i2c.c �ļ��У���� I2C_Start��I2C_Stop��I2C_Write��I2C_Read ��������
