2018/5/30
CSL\SWM220_StdPeriph_Driver\SWM220_gpio.c文件中添加确保引脚“读-改-写”操作原子性的GPIO_Atomic类型函数

2018/6/13
CSL\CMSIS\DeviceSupport\SWM220.h文件中，RTC_TypeDef添加校准用寄存器CALIBREFCNT、CALIBEN、CALIBST定义
CSL\SWM220_StdPeriph_Driver\SWM220_rtc.c文件中，RTC_AlarmSetup()函数添加语句：if(alarmStruct->AlarmIEn) NVIC_EnableIRQ(RTC_IRQn);
添加RTC校准演示例程RTC\SimplRTC_Calibrate

2018/6/27
CSL\SWM220_StdPeriph_Driver\SWM220_can.h文件中struct CAN_RXMessage的定义中添加format字段，表明接收到的这个消息是标准帧还是扩展帧
CSL\SWM220_StdPeriph_Driver\SWM220_can.c文件中CAN_Receive()函数中给CAN_RXMessage->format正确赋值
CAN\SimplCAN_RX_Interrupt例程中打印信息时打印出CAN_RXMessage->format字段

2018/8/17
CSL\SWM220_StdPeriph_Driver\SWM220_uart.c文件内UART_SetBaudrate()函数中，将：
UARTx->BAUD |= ((SystemCoreClock/16/baudrate) << UART_BAUD_BAUD_Pos);
改正为：
UARTx->BAUD |= ((SystemCoreClock/16/baudrate - 1) << UART_BAUD_BAUD_Pos);

2018/9/25
添加CSL\SWM220_StdPeriph_Driver\SWM220_sleep.c文件
CSL\SWM220_StdPeriph_Driver\SWM220_flash.c文件中AppToIsp()函数中__dsb(0)替换成__DSB()
CSL\CMSIS\DeviceSupport\SWM220.h文件中struct SYS_TypeDef定义中，将RESETST重命名为RSTST，添加RTCWK的位定义

2018/9/26
CSL\CMSIS\DeviceSupport\SWM220.h文件中struct RTC_TypeDef定义中LOAD寄存器添加位定义
CSL\SWM220_StdPeriph_Driver\SWM220_rtc.c文件中RTC_Init()、RTC_AlarmSetup()中RTC->LOAD寄存器操作修正

2018/10/29
CSL\SWM220_StdPeriph_Driver\SWM220_can.c文件中CAN_SetFilter32b()和CAN_SetFilter16b()函数中，将：
CANx->FILTER.AMR[0] = initStruct->FilterMask32b & 0xFF;
CANx->FILTER.AMR[1] = (initStruct->FilterMask32b >>  8) & 0xFF;
... ...
修正为：
CANx->FILTER.AMR[3] = initStruct->FilterMask32b & 0xFF;
CANx->FILTER.AMR[2] = (initStruct->FilterMask32b >>  8) & 0xFF;

2019/01/16
CSL\SWM220_StdPeriph_Driver\SWM220_dma.c文件中 DMA_CH_Config()函数中，将：
DMA->IF  = (1 << chn);		//清除中断标志
DMA->IE |= (1 << chn);
if(int_en)  DMA->IM &= ~(1 << chn);
else        DMA->IM |=  (1 << chn);
修正为：
DMA->IE = 0x35;			//即使不使能中断，也可以查询状态/标志
DMA_CH_INTClr(chn);		//清除中断标志
if(int_en) DMA_CH_INTEn(chn);
else	   DMA_CH_INTDis(chn);

2019/01/17
CSL\CMSIS\DeviceSupport\SWM220.h文件中，DMA->CR.WEN位位置改正

2019/03/04
CSL\SWM220_StdPeriph_Driver\SWM220_uart.c 文件中 UART_GetBaudrate() 中将：
return (UARTx->BAUD & UART_BAUD_BAUD_Msk);
修正为：
return SystemCoreClock/16/(((UARTx->BAUD & UART_BAUD_BAUD_Msk) >> UART_BAUD_BAUD_Pos) + 1);

CSL\SWM220_StdPeriph_Driver\SWM220_sleep.c 文件中，将 EnterStopMode1() 修正为：EnterStopMode()

2019/03/07
CSL\SWM220_StdPeriph_Driver\SWM220_adc.c 文件中 ADC_Init() 中将：
if(initStruct->clk_src == ADC_CLKSRC_XTAL_DIV4)
{
	PORT_Init(PORTD, PIN3, PORTD_PIN3_XTAL_IN,  1);
	PORT_Init(PORTD, PIN2, PORTD_PIN2_XTAL_OUT, 1);
	
	SYS->XTALCR = (1 << SYS_XTALCR_EN_Pos);
}
删除，若ADC使用晶振作为时钟源，而晶振又未被开启，可由用户在初始化ADC之前开启之

2019/04/18
CSL\CMSIS\DeviceSupport\SWM220.h 文件中，修正CAN->ECC寄存器位定义

2019/10/15
CSL\CMSIS\DeviceSupport\SWM220.h 文件中，UART->BAUD寄存器中的 RXD 位重命名为 RXDn

2019/11/07
CSL\SWM220_StdPeriph_Driver\SWM220_spi.c文件中添加SPI_INTRXHalfFullClr()、SPI_INTRXFullClr()、SPI_INTTXHalfFullClr()、SPI_INTTXEmptyClr()四个中断标志清除函数

2020/03/03
CSL\SWM220_StdPeriph_Driver\SWM220_i2c.c 文件中，添加 I2C_Start、I2C_Stop、I2C_Write、I2C_Read 函数定义
