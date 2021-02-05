#ifndef __SWM220_H__
#define __SWM220_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M0 Processor Exceptions Numbers **********************************************/
  NonINTMableInt_IRQn = -14,	/*!< 2 Non INTMable Interrupt								 */
  HardFault_IRQn	  = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt						 */
  SVCall_IRQn		  = -5,	 /*!< 11 Cortex-M0 SV Call Interrupt						     */
  PendSV_IRQn		  = -2,	 /*!< 14 Cortex-M0 Pend SV Interrupt						     */
  SysTick_IRQn		  = -1,	 /*!< 15 Cortex-M0 System Tick Interrupt					     */

/******  Cortex-M0 specific Interrupt Numbers ************************************************/
  GPIOA0_IRQn		  = 0,	  /*!< 														     */
  WDT_IRQn			  = 1,	  /*!< maximum of 32 Interrupts are possible	 			     */
  TIMR3_IRQn		  = 2,
  PWMNC_IRQn 		  = 3,
  ADC_IRQn			  = 4,
  TIMR2_IRQn		  = 5,
  RTC_IRQn			  = 6,
  GPIOA1_IRQn		  = 7,
  SPI1_IRQn		      = 8,
  GPIOD1_IRQn		  = 9,
  CAN_IRQn			  = 10,
  UART0_IRQn		  = 11,
  I2C0_IRQn			  = 12,
  SPI0_IRQn			  = 13,
  GPIOA_IRQn		  = 14,
  TIMR1_IRQn		  = 15,
  GPIOB0_IRQn		  = 16,
  UART1_IRQn		  = 17,
  GPIOB_IRQn		  = 18,
  TIMR0_IRQn		  = 19,
  GPIOB1_IRQn		  = 20,
  PWMHE_IRQn		  = 21,
  UART2_IRQn   		  = 22,
  BOD_IRQn	  		  = 23,
  I2C1_IRQn	   		  = 24,
  GPIOC0_IRQn	   	  = 25,
  GPIOC1_IRQn	   	  = 26,
  UART3_IRQn   		  = 27,
  PWMHALT_IRQn	  	  = 28,
  GPIOC_IRQn   		  = 29,
  GPIOD_IRQn		  = 30,
  DMA_IRQn			  = 31
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT		    0	   /*!< UART does not provide a MPU present or not	     */
#define __NVIC_PRIO_BITS		2	   /*!< UART Supports 2 Bits for the Priority Levels	 */
#define __Vendor_SysTickConfig  0	   /*!< Set to 1 if different SysTick Config is used	 */

#if   defined ( __CC_ARM )
  #pragma anon_unions
#endif

#include <stdio.h>
#include "core_cm0.h"				   /* Cortex-M0 processor and core peripherals		     */
#include "system_SWM220.h"


/******************************************************************************/
/*				Device Specific Peripheral registers structures			 */
/******************************************************************************/
typedef struct {
	__IO uint32_t CLKSEL;				    //Clock Select
	
		 uint32_t RESERVED;
	
	__IO uint32_t CLKEN;					//Clock Enable
	
		 uint32_t RESERVED2;
	
	__IO uint32_t SLEEP;
	
		 uint32_t RESERVED3[7];
	
	__IO uint32_t RTCWK;					//RTC Wakeup

		 uint32_t RESERVED4[51];
	
	__IO uint32_t CHIP_ID[4];
	
		 uint32_t RESERVED5[60];

	__IO uint32_t PAWKEN;				    //Port A Wakeup Enable
	__IO uint32_t PBWKEN;
	__IO uint32_t PCWKEN;
	__IO uint32_t PDWKEN;
	
		 uint32_t RESERVED6[2];
	
	__IO uint32_t PAWKSR;				    //Port A Wakeup Status Register��д1����
	__IO uint32_t PBWKSR;
	__IO uint32_t PCWKSR;
	__IO uint32_t PDWKSR;
	
		 uint32_t RESERVED7[(0x308-0x224)/4-1];
		 
	__IO uint32_t RSTST;
	
         uint32_t RESERVED8[(0x5000C000-0x40000308)/4-1];
    
    __IO uint32_t HRCCR;					//High speed RC Control Register
	
		 uint32_t RESERVED9[3];
	
	__IO uint32_t BODCR;
	
		 uint32_t RESERVED10[2];
	
	__IO uint32_t XTALCR;
	
	__IO uint32_t LRCCR;					//Low speed RC Control Register
	
	     uint32_t RESERVED11[47];
	
	__IO uint32_t TESTANA;
} SYS_TypeDef;


#define SYS_CLKSEL_LFCK_Pos			0		//Low Frequency Clock Source	0 LRC	1 XTAL
#define SYS_CLKSEL_LFCK_Msk			(0x01 << SYS_CLKSEL_LFCK_Pos)
#define SYS_CLKSEL_HFCK_Pos			1		//High Frequency Clock Source	0 HRC	1 HRC/4
#define SYS_CLKSEL_HFCK_Msk			(0x01 << SYS_CLKSEL_HFCK_Pos)
#define SYS_CLKSEL_SYS_Pos			2		//ϵͳʱ��ѡ��	0 LFCK	1 HFCK
#define SYS_CLKSEL_SYS_Msk			(0x01 << SYS_CLKSEL_SYS_Pos)
#define SYS_CLKSEL_RTC_Pos			4		//RTCУ׼�ο�ʱ��   0 XTAL	1 XTAL/2  2 XTAL/4  3 XTAL/8
#define SYS_CLKSEL_RTC_Msk			(0x03 << SYS_CLKSEL_RTC_Pos)

#define SYS_CLKEN_GPIOA_Pos			0
#define SYS_CLKEN_GPIOA_Msk			(0x01 << SYS_CLKEN_GPIOA_Pos)
#define SYS_CLKEN_GPIOB_Pos			1
#define SYS_CLKEN_GPIOB_Msk			(0x01 << SYS_CLKEN_GPIOB_Pos)
#define SYS_CLKEN_GPIOC_Pos			2
#define SYS_CLKEN_GPIOC_Msk			(0x01 << SYS_CLKEN_GPIOC_Pos)
#define SYS_CLKEN_GPIOD_Pos			3
#define SYS_CLKEN_GPIOD_Msk			(0x01 << SYS_CLKEN_GPIOD_Pos)
#define SYS_CLKEN_TIMR_Pos			6
#define SYS_CLKEN_TIMR_Msk			(0x01 << SYS_CLKEN_TIMR_Pos)
#define SYS_CLKEN_WDT_Pos			7
#define SYS_CLKEN_WDT_Msk			(0x01 << SYS_CLKEN_WDT_Pos)
#define SYS_CLKEN_ADC_Pos			8
#define SYS_CLKEN_ADC_Msk			(0x01 << SYS_CLKEN_ADC_Pos)
#define SYS_CLKEN_PWM_Pos			9
#define SYS_CLKEN_PWM_Msk			(0x01 << SYS_CLKEN_PWM_Pos)
#define SYS_CLKEN_RTC_Pos			10
#define SYS_CLKEN_RTC_Msk			(0x01 << SYS_CLKEN_RTC_Pos)
#define SYS_CLKEN_UART0_Pos			11
#define SYS_CLKEN_UART0_Msk			(0x01 << SYS_CLKEN_UART0_Pos)
#define SYS_CLKEN_UART1_Pos			12
#define SYS_CLKEN_UART1_Msk			(0x01 << SYS_CLKEN_UART1_Pos)
#define SYS_CLKEN_UART2_Pos			13
#define SYS_CLKEN_UART2_Msk			(0x01 << SYS_CLKEN_UART2_Pos)
#define SYS_CLKEN_UART3_Pos			14
#define SYS_CLKEN_UART3_Msk			(0x01 << SYS_CLKEN_UART3_Pos)
#define SYS_CLKEN_SPI1_Pos			15
#define SYS_CLKEN_SPI1_Msk			(0x01 << SYS_CLKEN_SPI1_Pos)
#define SYS_CLKEN_SPI0_Pos			16
#define SYS_CLKEN_SPI0_Msk			(0x01 << SYS_CLKEN_SPI0_Pos)
#define SYS_CLKEN_I2C0_Pos			17
#define SYS_CLKEN_I2C0_Msk			(0x01 << SYS_CLKEN_I2C0_Pos)
#define SYS_CLKEN_I2C1_Pos			18
#define SYS_CLKEN_I2C1_Msk			(0x01 << SYS_CLKEN_I2C1_Pos)
#define SYS_CLKEN_OSC_Pos			22
#define SYS_CLKEN_OSC_Msk			(0x01 << SYS_CLKEN_OSC_Pos)
#define SYS_CLKEN_CAN_Pos			25
#define SYS_CLKEN_CAN_Msk			(0x01 << SYS_CLKEN_CAN_Pos)
#define SYS_CLKEN_DIV_Pos			29
#define SYS_CLKEN_DIV_Msk			(0x01 << SYS_CLKEN_DIV_Pos)

#define SYS_SLEEP_SLEEP_Pos			0		//����λ��1��ϵͳ������SLEEPģʽ
#define SYS_SLEEP_SLEEP_Msk			(0x01 << SYS_SLEEP_SLEEP_Pos)
#define SYS_SLEEP_STOP_Pos			1		//����λ��1��ϵͳ������STOP ģʽ
#define SYS_SLEEP_STOP_Msk			(0x01 << SYS_SLEEP_STOP_Pos)

#define SYS_RTCWK_EN_Pos			0
#define SYS_RTCWK_EN_Msk			(0x01 << SYS_RTCWK_EN_Pos)
#define SYS_RTCWK_ST_Pos			1		//RTC����״̬��־λ��д1��
#define SYS_RTCWK_ST_Msk			(0x01 << SYS_RTCWK_ST_Pos)

#define SYS_RSTST_POR_Pos			0		//1 ��ʾ���ֹ�POR��λ��д1����
#define SYS_RSTST_POR_Msk			(0x01 << SYS_RSTST_POR_Pos)
#define SYS_RSTST_WDT_Pos			3		//1 ��ʾ���ֹ�WDT��λ��д1����
#define SYS_RSTST_WDT_Msk			(0x01 << SYS_RSTST_WDT_Pos)

#define SYS_HRCCR_EN_Pos			0		//High speed RC Enable
#define SYS_HRCCR_EN_Msk			(0x01 << SYS_HRCCR_EN_Pos)
#define SYS_HRCCR_DBL_Pos		    1		//Double Frequency	0 24MHz	1 48MHz
#define SYS_HRCCR_DBL_Msk		    (0x01 << SYS_HRCCR_DBL_Pos)

#define SYS_BODCR_INTEN_Pos			1		//BOD�ж�ʹ��
#define SYS_BODCR_INTEN_Msk			(0x01 << SYS_BODCR_INTEN_Pos)
#define SYS_BODCR_INTST_Pos			2		//BOD�ж�״̬��ֻ��
#define SYS_BODCR_INTST_Msk			(0x01 << SYS_BODCR_INTST_Pos)
#define SYS_BODCR_RSTLVL_Pos		4		//BOD��λ��ƽ��0 2.0V������λ   1 1.7V������λ
#define SYS_BODCR_RSTLVL_Msk		(0x01 << SYS_BODCR_RSTLVL_Pos)
#define SYS_BODCR_INTLVL_Pos		5		//BOD�жϵ�ƽ��0 2.7V�����ж�   1 2.3V�����ж�   2 2.0V�����ж�
#define SYS_BODCR_INTLVL_Msk		(0x03 << SYS_BODCR_INTLVL_Pos)

#define SYS_XTALCR_EN_Pos		    1
#define SYS_XTALCR_EN_Msk		    (0x01 << SYS_XTALCR_EN_Pos)

#define SYS_LRCCR_EN_Pos			0		//Low Speed RC Enable
#define SYS_LRCCR_EN_Msk			(0x01 << SYS_LRCCR_EN_Pos)




typedef struct {
	__IO uint32_t FUNSEL;
	
		 uint32_t RESERVED[127];
	
	__IO uint32_t PULLU;                    //PULLU[n]Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 ������ֹ
	
		 uint32_t RESERVED2[63];
	
	__IO uint32_t PULLD;                    //PULLD[n]Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 ������ֹ
	
		 uint32_t RESERVED3[63];
	
	__IO uint32_t OPEND;                    //OPEND[n]Ϊ PINn���� ��©ʹ��λ�� 1 ��©ʹ��	0 ��©��ֹ
	
		 uint32_t RESERVED4[127];
	
	__IO uint32_t INEN;                     //INEN[n] Ϊ PINn���� ����ʹ��λ�� 1 ����ʹ��	0 �����ֹ
} PORT_TypeDef;


#define PORT_FUNSEL_PIN0_Pos		0
#define PORT_FUNSEL_PIN0_Msk		(0x01 << PORT_FUNSEL_PIN0_Pos)
#define PORT_FUNSEL_PIN1_Pos		1
#define PORT_FUNSEL_PIN1_Msk		(0x01 << PORT_FUNSEL_PIN1_Pos)
#define PORT_FUNSEL_PIN2_Pos		2
#define PORT_FUNSEL_PIN2_Msk		(0x01 << PORT_FUNSEL_PIN2_Pos)
#define PORT_FUNSEL_PIN3_Pos		3
#define PORT_FUNSEL_PIN3_Msk		(0x01 << PORT_FUNSEL_PIN3_Pos)
#define PORT_FUNSEL_PIN4_Pos		4
#define PORT_FUNSEL_PIN4_Msk		(0x01 << PORT_FUNSEL_PIN4_Pos)
#define PORT_FUNSEL_PIN5_Pos		5
#define PORT_FUNSEL_PIN5_Msk		(0x01 << PORT_FUNSEL_PIN5_Pos)
#define PORT_FUNSEL_PIN6_Pos		6
#define PORT_FUNSEL_PIN6_Msk		(0x01 << PORT_FUNSEL_PIN6_Pos)
#define PORT_FUNSEL_PIN7_Pos		7
#define PORT_FUNSEL_PIN7_Msk		(0x01 << PORT_FUNSEL_PIN7_Pos)
#define PORT_FUNSEL_PIN8_Pos		8
#define PORT_FUNSEL_PIN8_Msk		(0x01 << PORT_FUNSEL_PIN8_Pos)
#define PORT_FUNSEL_PIN9_Pos		9
#define PORT_FUNSEL_PIN9_Msk		(0x01 << PORT_FUNSEL_PIN9_Pos)
#define PORT_FUNSEL_PIN10_Pos		10
#define PORT_FUNSEL_PIN10_Msk		(0x01 << PORT_FUNSEL_PIN10_Pos)
#define PORT_FUNSEL_PIN11_Pos		11
#define PORT_FUNSEL_PIN11_Msk		(0x01 << PORT_FUNSEL_PIN11_Pos)
#define PORT_FUNSEL_PIN12_Pos		12
#define PORT_FUNSEL_PIN12_Msk		(0x01 << PORT_FUNSEL_PIN12_Pos)
#define PORT_FUNSEL_PIN13_Pos		13
#define PORT_FUNSEL_PIN13_Msk		(0x01 << PORT_FUNSEL_PIN13_Pos)
#define PORT_FUNSEL_PIN14_Pos		14
#define PORT_FUNSEL_PIN14_Msk		(0x01 << PORT_FUNSEL_PIN14_Pos)
#define PORT_FUNSEL_PIN15_Pos		15
#define PORT_FUNSEL_PIN15_Msk		(0x01 << PORT_FUNSEL_PIN15_Pos)

#define PORT_PULLU_PIN0_Pos			0
#define PORT_PULLU_PIN0_Msk			(0x01 << PORT_PULLU_PIN0_Pos)
#define PORT_PULLU_PIN1_Pos			1
#define PORT_PULLU_PIN1_Msk			(0x01 << PORT_PULLU_PIN1_Pos)
#define PORT_PULLU_PIN2_Pos			2
#define PORT_PULLU_PIN2_Msk			(0x01 << PORT_PULLU_PIN2_Pos)
#define PORT_PULLU_PIN3_Pos			3
#define PORT_PULLU_PIN3_Msk			(0x01 << PORT_PULLU_PIN3_Pos)
#define PORT_PULLU_PIN4_Pos			4
#define PORT_PULLU_PIN4_Msk			(0x01 << PORT_PULLU_PIN4_Pos)
#define PORT_PULLU_PIN5_Pos			5
#define PORT_PULLU_PIN5_Msk			(0x01 << PORT_PULLU_PIN5_Pos)
#define PORT_PULLU_PIN6_Pos			6
#define PORT_PULLU_PIN6_Msk			(0x01 << PORT_PULLU_PIN6_Pos)
#define PORT_PULLU_PIN7_Pos			7
#define PORT_PULLU_PIN7_Msk			(0x01 << PORT_PULLU_PIN7_Pos)
#define PORT_PULLU_PIN8_Pos			8
#define PORT_PULLU_PIN8_Msk			(0x01 << PORT_PULLU_PIN8_Pos)
#define PORT_PULLU_PIN9_Pos			9
#define PORT_PULLU_PIN9_Msk			(0x01 << PORT_PULLU_PIN9_Pos)
#define PORT_PULLU_PIN10_Pos		10
#define PORT_PULLU_PIN10_Msk		(0x01 << PORT_PULLU_PIN10_Pos)
#define PORT_PULLU_PIN11_Pos		11
#define PORT_PULLU_PIN11_Msk		(0x01 << PORT_PULLU_PIN11_Pos)
#define PORT_PULLU_PIN12_Pos		12
#define PORT_PULLU_PIN12_Msk		(0x01 << PORT_PULLU_PIN12_Pos)
#define PORT_PULLU_PIN13_Pos		13
#define PORT_PULLU_PIN13_Msk		(0x01 << PORT_PULLU_PIN13_Pos)
#define PORT_PULLU_PIN14_Pos		14
#define PORT_PULLU_PIN14_Msk		(0x01 << PORT_PULLU_PIN14_Pos)
#define PORT_PULLU_PIN15_Pos		15
#define PORT_PULLU_PIN15_Msk		(0x01 << PORT_PULLU_PIN15_Pos)

#define PORT_PULLD_PIN0_Pos			0
#define PORT_PULLD_PIN0_Msk			(0x01 << PORT_PULLD_PIN0_Pos)
#define PORT_PULLD_PIN1_Pos			1
#define PORT_PULLD_PIN1_Msk			(0x01 << PORT_PULLD_PIN1_Pos)
#define PORT_PULLD_PIN2_Pos			2
#define PORT_PULLD_PIN2_Msk			(0x01 << PORT_PULLD_PIN2_Pos)
#define PORT_PULLD_PIN3_Pos			3
#define PORT_PULLD_PIN3_Msk			(0x01 << PORT_PULLD_PIN3_Pos)
#define PORT_PULLD_PIN4_Pos			4
#define PORT_PULLD_PIN4_Msk			(0x01 << PORT_PULLD_PIN4_Pos)
#define PORT_PULLD_PIN5_Pos			5
#define PORT_PULLD_PIN5_Msk			(0x01 << PORT_PULLD_PIN5_Pos)
#define PORT_PULLD_PIN6_Pos			6
#define PORT_PULLD_PIN6_Msk			(0x01 << PORT_PULLD_PIN6_Pos)
#define PORT_PULLD_PIN7_Pos			7
#define PORT_PULLD_PIN7_Msk			(0x01 << PORT_PULLD_PIN7_Pos)
#define PORT_PULLD_PIN8_Pos			8
#define PORT_PULLD_PIN8_Msk			(0x01 << PORT_PULLD_PIN8_Pos)
#define PORT_PULLD_PIN9_Pos			9
#define PORT_PULLD_PIN9_Msk			(0x01 << PORT_PULLD_PIN9_Pos)
#define PORT_PULLD_PIN10_Pos		10
#define PORT_PULLD_PIN10_Msk		(0x01 << PORT_PULLD_PIN10_Pos)
#define PORT_PULLD_PIN11_Pos		11
#define PORT_PULLD_PIN11_Msk		(0x01 << PORT_PULLD_PIN11_Pos)
#define PORT_PULLD_PIN12_Pos		12
#define PORT_PULLD_PIN12_Msk		(0x01 << PORT_PULLD_PIN12_Pos)
#define PORT_PULLD_PIN13_Pos		13
#define PORT_PULLD_PIN13_Msk		(0x01 << PORT_PULLD_PIN13_Pos)
#define PORT_PULLD_PIN14_Pos		14
#define PORT_PULLD_PIN14_Msk		(0x01 << PORT_PULLD_PIN14_Pos)
#define PORT_PULLD_PIN15_Pos		15
#define PORT_PULLD_PIN15_Msk		(0x01 << PORT_PULLD_PIN15_Pos)

#define PORT_OPEND_PIN0_Pos			0
#define PORT_OPEND_PIN0_Msk			(0x01 << PORT_OPEND_PIN0_Pos)
#define PORT_OPEND_PIN1_Pos			1
#define PORT_OPEND_PIN1_Msk			(0x01 << PORT_OPEND_PIN1_Pos)
#define PORT_OPEND_PIN2_Pos			2
#define PORT_OPEND_PIN2_Msk			(0x01 << PORT_OPEND_PIN2_Pos)
#define PORT_OPEND_PIN3_Pos			3
#define PORT_OPEND_PIN3_Msk			(0x01 << PORT_OPEND_PIN3_Pos)
#define PORT_OPEND_PIN4_Pos			4
#define PORT_OPEND_PIN4_Msk			(0x01 << PORT_OPEND_PIN4_Pos)
#define PORT_OPEND_PIN5_Pos			5
#define PORT_OPEND_PIN5_Msk			(0x01 << PORT_OPEND_PIN5_Pos)
#define PORT_OPEND_PIN6_Pos			6
#define PORT_OPEND_PIN6_Msk			(0x01 << PORT_OPEND_PIN6_Pos)
#define PORT_OPEND_PIN7_Pos			7
#define PORT_OPEND_PIN7_Msk			(0x01 << PORT_OPEND_PIN7_Pos)
#define PORT_OPEND_PIN8_Pos			8
#define PORT_OPEND_PIN8_Msk			(0x01 << PORT_OPEND_PIN8_Pos)
#define PORT_OPEND_PIN9_Pos			9
#define PORT_OPEND_PIN9_Msk			(0x01 << PORT_OPEND_PIN9_Pos)
#define PORT_OPEND_PIN10_Pos		10
#define PORT_OPEND_PIN10_Msk		(0x01 << PORT_OPEND_PIN10_Pos)
#define PORT_OPEND_PIN11_Pos		11
#define PORT_OPEND_PIN11_Msk		(0x01 << PORT_OPEND_PIN11_Pos)
#define PORT_OPEND_PIN12_Pos		12
#define PORT_OPEND_PIN12_Msk		(0x01 << PORT_OPEND_PIN12_Pos)
#define PORT_OPEND_PIN13_Pos		13
#define PORT_OPEND_PIN13_Msk		(0x01 << PORT_OPEND_PIN13_Pos)
#define PORT_OPEND_PIN14_Pos		14
#define PORT_OPEND_PIN14_Msk		(0x01 << PORT_OPEND_PIN14_Pos)
#define PORT_OPEND_PIN15_Pos		15
#define PORT_OPEND_PIN15_Msk		(0x01 << PORT_OPEND_PIN15_Pos)

#define PORT_INEN_PIN0_Pos			0
#define PORT_INEN_PIN0_Msk			(0x01 << PORT_INEN_PIN0_Pos)
#define PORT_INEN_PIN1_Pos			1
#define PORT_INEN_PIN1_Msk			(0x01 << PORT_INEN_PIN1_Pos)
#define PORT_INEN_PIN2_Pos			2
#define PORT_INEN_PIN2_Msk			(0x01 << PORT_INEN_PIN2_Pos)
#define PORT_INEN_PIN3_Pos			3
#define PORT_INEN_PIN3_Msk			(0x01 << PORT_INEN_PIN3_Pos)
#define PORT_INEN_PIN4_Pos			4
#define PORT_INEN_PIN4_Msk			(0x01 << PORT_INEN_PIN4_Pos)
#define PORT_INEN_PIN5_Pos			5
#define PORT_INEN_PIN5_Msk			(0x01 << PORT_INEN_PIN5_Pos)
#define PORT_INEN_PIN6_Pos			6
#define PORT_INEN_PIN6_Msk			(0x01 << PORT_INEN_PIN6_Pos)
#define PORT_INEN_PIN7_Pos			7
#define PORT_INEN_PIN7_Msk			(0x01 << PORT_INEN_PIN7_Pos)
#define PORT_INEN_PIN8_Pos			8
#define PORT_INEN_PIN8_Msk			(0x01 << PORT_INEN_PIN8_Pos)
#define PORT_INEN_PIN9_Pos			9
#define PORT_INEN_PIN9_Msk			(0x01 << PORT_INEN_PIN9_Pos)
#define PORT_INEN_PIN10_Pos			10
#define PORT_INEN_PIN10_Msk			(0x01 << PORT_INEN_PIN10_Pos)
#define PORT_INEN_PIN11_Pos			11
#define PORT_INEN_PIN11_Msk			(0x01 << PORT_INEN_PIN11_Pos)
#define PORT_INEN_PIN12_Pos			12
#define PORT_INEN_PIN12_Msk			(0x01 << PORT_INEN_PIN12_Pos)
#define PORT_INEN_PIN13_Pos			13
#define PORT_INEN_PIN13_Msk			(0x01 << PORT_INEN_PIN13_Pos)
#define PORT_INEN_PIN14_Pos			14
#define PORT_INEN_PIN14_Msk			(0x01 << PORT_INEN_PIN14_Pos)
#define PORT_INEN_PIN15_Pos			15
#define PORT_INEN_PIN15_Msk			(0x01 << PORT_INEN_PIN15_Pos)




typedef struct {
	  union {
    __IO uint32_t DATA;
    struct {
      __IO uint32_t DAT_0: 1;
      __IO uint32_t DAT_1: 1;
      __IO uint32_t DAT_2: 1;
      __IO uint32_t DAT_3: 1;
      __IO uint32_t DAT_4: 1;
      __IO uint32_t DAT_5: 1;
      __IO uint32_t DAT_6: 1;
      __IO uint32_t DAT_7: 1;
      __IO uint32_t DAT_8: 1;
      __IO uint32_t DAT_9: 1;
      __IO uint32_t DAT_10:1;
      __IO uint32_t DAT_11:1;
      __IO uint32_t DAT_12:1;
      __IO uint32_t DAT_13:1;
      __IO uint32_t DAT_14:1;
      __IO uint32_t DAT_15:1;
      __I  uint32_t RESERVE:16; 
    } DAT;
  };
#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7
#define PIN8    8
#define PIN9    9
#define PIN10   10
#define PIN11   11
#define PIN12   12
#define PIN13   13
#define PIN14   14
#define PIN15   15

	__IO uint32_t DIR;					    //0 ����	1 ���

	__IO uint32_t INTLVLTRG;				//Interrupt Level Trigger  1 ��ƽ�����ж�	0 ���ش����ж�

	__IO uint32_t INTBE;					//Both Edge����INTLVLTRG��Ϊ���ش����ж�ʱ����λ��1��ʾ�����غ��½��ض������жϣ���0ʱ����������INTRISEENѡ��

	__IO uint32_t INTRISEEN;				//Interrupt Rise Edge Enable   1 ������/�ߵ�ƽ�����ж�	0 �½���/�͵�ƽ�����ж�

	__IO uint32_t INTEN;					//1 �ж�ʹ��	0 �жϽ�ֹ

	__IO uint32_t INTRAWSTAT;			    //�жϼ�ⵥԪ�Ƿ��⵽�˴����жϵ����� 1 ��⵽���жϴ�������	0 û�м�⵽�жϴ�������

	__IO uint32_t INTSTAT;				    //INTSTAT.PIN0 = INTRAWSTAT.PIN0 & INTEN.PIN0

	__IO uint32_t INTCLR;				    //д1����жϱ�־��ֻ�Ա��ش����ж�����
	
	     uint32_t RESERVED[7];
	
	__IO uint32_t DATAPIN0;					//PIN0���ŵ�DATA�Ĵ������������Ŷ�Ӧ����32λ�Ĵ���������ʵ��ԭ��д����
	__IO uint32_t DATAPIN1;
	__IO uint32_t DATAPIN2;
	__IO uint32_t DATAPIN3;
	__IO uint32_t DATAPIN4;
	__IO uint32_t DATAPIN5;
	__IO uint32_t DATAPIN6;
	__IO uint32_t DATAPIN7;
	__IO uint32_t DATAPIN8;
	__IO uint32_t DATAPIN9;
	__IO uint32_t DATAPIN10;
	__IO uint32_t DATAPIN11;
	__IO uint32_t DATAPIN12;
	__IO uint32_t DATAPIN13;
	__IO uint32_t DATAPIN14;
	__IO uint32_t DATAPIN15;
} GPIO_TypeDef;


#define GPIO_DATA_PIN0_Pos			0
#define GPIO_DATA_PIN0_Msk			(0x01 << GPIO_DATA_PIN0_Pos)
#define GPIO_DATA_PIN1_Pos			1
#define GPIO_DATA_PIN1_Msk			(0x01 << GPIO_DATA_PIN1_Pos)
#define GPIO_DATA_PIN2_Pos			2
#define GPIO_DATA_PIN2_Msk			(0x01 << GPIO_DATA_PIN2_Pos)
#define GPIO_DATA_PIN3_Pos			3
#define GPIO_DATA_PIN3_Msk			(0x01 << GPIO_DATA_PIN3_Pos)
#define GPIO_DATA_PIN4_Pos			4
#define GPIO_DATA_PIN4_Msk			(0x01 << GPIO_DATA_PIN4_Pos)
#define GPIO_DATA_PIN5_Pos			5
#define GPIO_DATA_PIN5_Msk			(0x01 << GPIO_DATA_PIN5_Pos)
#define GPIO_DATA_PIN6_Pos			6
#define GPIO_DATA_PIN6_Msk			(0x01 << GPIO_DATA_PIN6_Pos)
#define GPIO_DATA_PIN7_Pos			7
#define GPIO_DATA_PIN7_Msk			(0x01 << GPIO_DATA_PIN7_Pos)
#define GPIO_DATA_PIN8_Pos			8
#define GPIO_DATA_PIN8_Msk			(0x01 << GPIO_DATA_PIN8_Pos)
#define GPIO_DATA_PIN9_Pos			9
#define GPIO_DATA_PIN9_Msk			(0x01 << GPIO_DATA_PIN9_Pos)
#define GPIO_DATA_PIN10_Pos			10
#define GPIO_DATA_PIN10_Msk			(0x01 << GPIO_DATA_PIN10_Pos)
#define GPIO_DATA_PIN11_Pos			11
#define GPIO_DATA_PIN11_Msk			(0x01 << GPIO_DATA_PIN11_Pos)
#define GPIO_DATA_PIN12_Pos			12
#define GPIO_DATA_PIN12_Msk			(0x01 << GPIO_DATA_PIN12_Pos)
#define GPIO_DATA_PIN13_Pos			13
#define GPIO_DATA_PIN13_Msk			(0x01 << GPIO_DATA_PIN13_Pos)
#define GPIO_DATA_PIN14_Pos			14
#define GPIO_DATA_PIN14_Msk			(0x01 << GPIO_DATA_PIN14_Pos)
#define GPIO_DATA_PIN15_Pos			15
#define GPIO_DATA_PIN15_Msk			(0x01 << GPIO_DATA_PIN15_Pos)

#define GPIO_DIR_PIN0_Pos			0
#define GPIO_DIR_PIN0_Msk			(0x01 << GPIO_DIR_PIN0_Pos)
#define GPIO_DIR_PIN1_Pos			1
#define GPIO_DIR_PIN1_Msk			(0x01 << GPIO_DIR_PIN1_Pos)
#define GPIO_DIR_PIN2_Pos			2
#define GPIO_DIR_PIN2_Msk			(0x01 << GPIO_DIR_PIN2_Pos)
#define GPIO_DIR_PIN3_Pos			3
#define GPIO_DIR_PIN3_Msk			(0x01 << GPIO_DIR_PIN3_Pos)
#define GPIO_DIR_PIN4_Pos			4
#define GPIO_DIR_PIN4_Msk			(0x01 << GPIO_DIR_PIN4_Pos)
#define GPIO_DIR_PIN5_Pos			5
#define GPIO_DIR_PIN5_Msk			(0x01 << GPIO_DIR_PIN5_Pos)
#define GPIO_DIR_PIN6_Pos			6
#define GPIO_DIR_PIN6_Msk			(0x01 << GPIO_DIR_PIN6_Pos)
#define GPIO_DIR_PIN7_Pos			7
#define GPIO_DIR_PIN7_Msk			(0x01 << GPIO_DIR_PIN7_Pos)
#define GPIO_DIR_PIN8_Pos			8
#define GPIO_DIR_PIN8_Msk			(0x01 << GPIO_DIR_PIN8_Pos)
#define GPIO_DIR_PIN9_Pos			9
#define GPIO_DIR_PIN9_Msk			(0x01 << GPIO_DIR_PIN9_Pos)
#define GPIO_DIR_PIN10_Pos			10
#define GPIO_DIR_PIN10_Msk			(0x01 << GPIO_DIR_PIN10_Pos)
#define GPIO_DIR_PIN11_Pos			11
#define GPIO_DIR_PIN11_Msk			(0x01 << GPIO_DIR_PIN11_Pos)
#define GPIO_DIR_PIN12_Pos			12
#define GPIO_DIR_PIN12_Msk			(0x01 << GPIO_DIR_PIN12_Pos)
#define GPIO_DIR_PIN13_Pos			13
#define GPIO_DIR_PIN13_Msk			(0x01 << GPIO_DIR_PIN13_Pos)
#define GPIO_DIR_PIN14_Pos			14
#define GPIO_DIR_PIN14_Msk			(0x01 << GPIO_DIR_PIN14_Pos)
#define GPIO_DIR_PIN15_Pos			15
#define GPIO_DIR_PIN15_Msk			(0x01 << GPIO_DIR_PIN15_Pos)

#define GPIO_INTLVLTRG_PIN0_Pos		0
#define GPIO_INTLVLTRG_PIN0_Msk		(0x01 << GPIO_INTLVLTRG_PIN0_Pos)
#define GPIO_INTLVLTRG_PIN1_Pos		1
#define GPIO_INTLVLTRG_PIN1_Msk		(0x01 << GPIO_INTLVLTRG_PIN1_Pos)
#define GPIO_INTLVLTRG_PIN2_Pos		2
#define GPIO_INTLVLTRG_PIN2_Msk		(0x01 << GPIO_INTLVLTRG_PIN2_Pos)
#define GPIO_INTLVLTRG_PIN3_Pos		3
#define GPIO_INTLVLTRG_PIN3_Msk		(0x01 << GPIO_INTLVLTRG_PIN3_Pos)
#define GPIO_INTLVLTRG_PIN4_Pos		4
#define GPIO_INTLVLTRG_PIN4_Msk		(0x01 << GPIO_INTLVLTRG_PIN4_Pos)
#define GPIO_INTLVLTRG_PIN5_Pos		5
#define GPIO_INTLVLTRG_PIN5_Msk		(0x01 << GPIO_INTLVLTRG_PIN5_Pos)
#define GPIO_INTLVLTRG_PIN6_Pos		6
#define GPIO_INTLVLTRG_PIN6_Msk		(0x01 << GPIO_INTLVLTRG_PIN6_Pos)
#define GPIO_INTLVLTRG_PIN7_Pos		7
#define GPIO_INTLVLTRG_PIN7_Msk		(0x01 << GPIO_INTLVLTRG_PIN7_Pos)
#define GPIO_INTLVLTRG_PIN8_Pos		8
#define GPIO_INTLVLTRG_PIN8_Msk		(0x01 << GPIO_INTLVLTRG_PIN8_Pos)
#define GPIO_INTLVLTRG_PIN9_Pos		9
#define GPIO_INTLVLTRG_PIN9_Msk		(0x01 << GPIO_INTLVLTRG_PIN9_Pos)
#define GPIO_INTLVLTRG_PIN10_Pos	10
#define GPIO_INTLVLTRG_PIN10_Msk	(0x01 << GPIO_INTLVLTRG_PIN10_Pos)
#define GPIO_INTLVLTRG_PIN11_Pos	11
#define GPIO_INTLVLTRG_PIN11_Msk	(0x01 << GPIO_INTLVLTRG_PIN11_Pos)
#define GPIO_INTLVLTRG_PIN12_Pos	12
#define GPIO_INTLVLTRG_PIN12_Msk	(0x01 << GPIO_INTLVLTRG_PIN12_Pos)
#define GPIO_INTLVLTRG_PIN13_Pos	13
#define GPIO_INTLVLTRG_PIN13_Msk	(0x01 << GPIO_INTLVLTRG_PIN13_Pos)
#define GPIO_INTLVLTRG_PIN14_Pos	14
#define GPIO_INTLVLTRG_PIN14_Msk	(0x01 << GPIO_INTLVLTRG_PIN14_Pos)
#define GPIO_INTLVLTRG_PIN15_Pos	15
#define GPIO_INTLVLTRG_PIN15_Msk	(0x01 << GPIO_INTLVLTRG_PIN15_Pos)

#define GPIO_INTBE_PIN0_Pos			0
#define GPIO_INTBE_PIN0_Msk			(0x01 << GPIO_INTBE_PIN0_Pos)
#define GPIO_INTBE_PIN1_Pos			1
#define GPIO_INTBE_PIN1_Msk			(0x01 << GPIO_INTBE_PIN1_Pos)
#define GPIO_INTBE_PIN2_Pos			2
#define GPIO_INTBE_PIN2_Msk			(0x01 << GPIO_INTBE_PIN2_Pos)
#define GPIO_INTBE_PIN3_Pos			3
#define GPIO_INTBE_PIN3_Msk			(0x01 << GPIO_INTBE_PIN3_Pos)
#define GPIO_INTBE_PIN4_Pos			4
#define GPIO_INTBE_PIN4_Msk			(0x01 << GPIO_INTBE_PIN4_Pos)
#define GPIO_INTBE_PIN5_Pos			5
#define GPIO_INTBE_PIN5_Msk			(0x01 << GPIO_INTBE_PIN5_Pos)
#define GPIO_INTBE_PIN6_Pos			6
#define GPIO_INTBE_PIN6_Msk			(0x01 << GPIO_INTBE_PIN6_Pos)
#define GPIO_INTBE_PIN7_Pos			7
#define GPIO_INTBE_PIN7_Msk			(0x01 << GPIO_INTBE_PIN7_Pos)
#define GPIO_INTBE_PIN8_Pos			8
#define GPIO_INTBE_PIN8_Msk			(0x01 << GPIO_INTBE_PIN8_Pos)
#define GPIO_INTBE_PIN9_Pos			9
#define GPIO_INTBE_PIN9_Msk			(0x01 << GPIO_INTBE_PIN9_Pos)
#define GPIO_INTBE_PIN10_Pos		10
#define GPIO_INTBE_PIN10_Msk		(0x01 << GPIO_INTBE_PIN10_Pos)
#define GPIO_INTBE_PIN11_Pos		11
#define GPIO_INTBE_PIN11_Msk		(0x01 << GPIO_INTBE_PIN11_Pos)
#define GPIO_INTBE_PIN12_Pos		12
#define GPIO_INTBE_PIN12_Msk		(0x01 << GPIO_INTBE_PIN12_Pos)
#define GPIO_INTBE_PIN13_Pos		13
#define GPIO_INTBE_PIN13_Msk		(0x01 << GPIO_INTBE_PIN13_Pos)
#define GPIO_INTBE_PIN14_Pos		14
#define GPIO_INTBE_PIN14_Msk		(0x01 << GPIO_INTBE_PIN14_Pos)
#define GPIO_INTBE_PIN15_Pos		15
#define GPIO_INTBE_PIN15_Msk		(0x01 << GPIO_INTBE_PIN15_Pos)

#define GPIO_INTRISEEN_PIN0_Pos		0
#define GPIO_INTRISEEN_PIN0_Msk		(0x01 << GPIO_INTRISEEN_PIN0_Pos)
#define GPIO_INTRISEEN_PIN1_Pos		1
#define GPIO_INTRISEEN_PIN1_Msk		(0x01 << GPIO_INTRISEEN_PIN1_Pos)
#define GPIO_INTRISEEN_PIN2_Pos		2
#define GPIO_INTRISEEN_PIN2_Msk		(0x01 << GPIO_INTRISEEN_PIN2_Pos)
#define GPIO_INTRISEEN_PIN3_Pos		3
#define GPIO_INTRISEEN_PIN3_Msk		(0x01 << GPIO_INTRISEEN_PIN3_Pos)
#define GPIO_INTRISEEN_PIN4_Pos		4
#define GPIO_INTRISEEN_PIN4_Msk		(0x01 << GPIO_INTRISEEN_PIN4_Pos)
#define GPIO_INTRISEEN_PIN5_Pos		5
#define GPIO_INTRISEEN_PIN5_Msk		(0x01 << GPIO_INTRISEEN_PIN5_Pos)
#define GPIO_INTRISEEN_PIN6_Pos		6
#define GPIO_INTRISEEN_PIN6_Msk		(0x01 << GPIO_INTRISEEN_PIN6_Pos)
#define GPIO_INTRISEEN_PIN7_Pos		7
#define GPIO_INTRISEEN_PIN7_Msk		(0x01 << GPIO_INTRISEEN_PIN7_Pos)
#define GPIO_INTRISEEN_PIN8_Pos		8
#define GPIO_INTRISEEN_PIN8_Msk		(0x01 << GPIO_INTRISEEN_PIN8_Pos)
#define GPIO_INTRISEEN_PIN9_Pos		9
#define GPIO_INTRISEEN_PIN9_Msk		(0x01 << GPIO_INTRISEEN_PIN9_Pos)
#define GPIO_INTRISEEN_PIN10_Pos	10
#define GPIO_INTRISEEN_PIN10_Msk	(0x01 << GPIO_INTRISEEN_PIN10_Pos)
#define GPIO_INTRISEEN_PIN11_Pos	11
#define GPIO_INTRISEEN_PIN11_Msk	(0x01 << GPIO_INTRISEEN_PIN11_Pos)
#define GPIO_INTRISEEN_PIN12_Pos	12
#define GPIO_INTRISEEN_PIN12_Msk	(0x01 << GPIO_INTRISEEN_PIN12_Pos)
#define GPIO_INTRISEEN_PIN13_Pos	13
#define GPIO_INTRISEEN_PIN13_Msk	(0x01 << GPIO_INTRISEEN_PIN13_Pos)
#define GPIO_INTRISEEN_PIN14_Pos	14
#define GPIO_INTRISEEN_PIN14_Msk	(0x01 << GPIO_INTRISEEN_PIN14_Pos)
#define GPIO_INTRISEEN_PIN15_Pos	15
#define GPIO_INTRISEEN_PIN15_Msk	(0x01 << GPIO_INTRISEEN_PIN15_Pos)

#define GPIO_INTEN_PIN0_Pos			0
#define GPIO_INTEN_PIN0_Msk			(0x01 << GPIO_INTEN_PIN0_Pos)
#define GPIO_INTEN_PIN1_Pos			1
#define GPIO_INTEN_PIN1_Msk			(0x01 << GPIO_INTEN_PIN1_Pos)
#define GPIO_INTEN_PIN2_Pos			2
#define GPIO_INTEN_PIN2_Msk			(0x01 << GPIO_INTEN_PIN2_Pos)
#define GPIO_INTEN_PIN3_Pos			3
#define GPIO_INTEN_PIN3_Msk			(0x01 << GPIO_INTEN_PIN3_Pos)
#define GPIO_INTEN_PIN4_Pos			4
#define GPIO_INTEN_PIN4_Msk			(0x01 << GPIO_INTEN_PIN4_Pos)
#define GPIO_INTEN_PIN5_Pos			5
#define GPIO_INTEN_PIN5_Msk			(0x01 << GPIO_INTEN_PIN5_Pos)
#define GPIO_INTEN_PIN6_Pos			6
#define GPIO_INTEN_PIN6_Msk			(0x01 << GPIO_INTEN_PIN6_Pos)
#define GPIO_INTEN_PIN7_Pos			7
#define GPIO_INTEN_PIN7_Msk			(0x01 << GPIO_INTEN_PIN7_Pos)
#define GPIO_INTEN_PIN8_Pos			8
#define GPIO_INTEN_PIN8_Msk			(0x01 << GPIO_INTEN_PIN8_Pos)
#define GPIO_INTEN_PIN9_Pos			9
#define GPIO_INTEN_PIN9_Msk			(0x01 << GPIO_INTEN_PIN9_Pos)
#define GPIO_INTEN_PIN10_Pos		10
#define GPIO_INTEN_PIN10_Msk		(0x01 << GPIO_INTEN_PIN10_Pos)
#define GPIO_INTEN_PIN11_Pos		11
#define GPIO_INTEN_PIN11_Msk		(0x01 << GPIO_INTEN_PIN11_Pos)
#define GPIO_INTEN_PIN12_Pos		12
#define GPIO_INTEN_PIN12_Msk		(0x01 << GPIO_INTEN_PIN12_Pos)
#define GPIO_INTEN_PIN13_Pos		13
#define GPIO_INTEN_PIN13_Msk		(0x01 << GPIO_INTEN_PIN13_Pos)
#define GPIO_INTEN_PIN14_Pos		14
#define GPIO_INTEN_PIN14_Msk		(0x01 << GPIO_INTEN_PIN14_Pos)
#define GPIO_INTEN_PIN15_Pos		15
#define GPIO_INTEN_PIN15_Msk		(0x01 << GPIO_INTEN_PIN15_Pos)

#define GPIO_INTRAWSTAT_PIN0_Pos	0
#define GPIO_INTRAWSTAT_PIN0_Msk	(0x01 << GPIO_INTRAWSTAT_PIN0_Pos)
#define GPIO_INTRAWSTAT_PIN1_Pos	1
#define GPIO_INTRAWSTAT_PIN1_Msk	(0x01 << GPIO_INTRAWSTAT_PIN1_Pos)
#define GPIO_INTRAWSTAT_PIN2_Pos	2
#define GPIO_INTRAWSTAT_PIN2_Msk	(0x01 << GPIO_INTRAWSTAT_PIN2_Pos)
#define GPIO_INTRAWSTAT_PIN3_Pos	3
#define GPIO_INTRAWSTAT_PIN3_Msk	(0x01 << GPIO_INTRAWSTAT_PIN3_Pos)
#define GPIO_INTRAWSTAT_PIN4_Pos	4
#define GPIO_INTRAWSTAT_PIN4_Msk	(0x01 << GPIO_INTRAWSTAT_PIN4_Pos)
#define GPIO_INTRAWSTAT_PIN5_Pos	5
#define GPIO_INTRAWSTAT_PIN5_Msk	(0x01 << GPIO_INTRAWSTAT_PIN5_Pos)
#define GPIO_INTRAWSTAT_PIN6_Pos	6
#define GPIO_INTRAWSTAT_PIN6_Msk	(0x01 << GPIO_INTRAWSTAT_PIN6_Pos)
#define GPIO_INTRAWSTAT_PIN7_Pos	7
#define GPIO_INTRAWSTAT_PIN7_Msk	(0x01 << GPIO_INTRAWSTAT_PIN7_Pos)
#define GPIO_INTRAWSTAT_PIN8_Pos	8
#define GPIO_INTRAWSTAT_PIN8_Msk	(0x01 << GPIO_INTRAWSTAT_PIN8_Pos)
#define GPIO_INTRAWSTAT_PIN9_Pos	9
#define GPIO_INTRAWSTAT_PIN9_Msk	(0x01 << GPIO_INTRAWSTAT_PIN9_Pos)
#define GPIO_INTRAWSTAT_PIN10_Pos	10
#define GPIO_INTRAWSTAT_PIN10_Msk	(0x01 << GPIO_INTRAWSTAT_PIN10_Pos)
#define GPIO_INTRAWSTAT_PIN11_Pos	11
#define GPIO_INTRAWSTAT_PIN11_Msk	(0x01 << GPIO_INTRAWSTAT_PIN11_Pos)
#define GPIO_INTRAWSTAT_PIN12_Pos	12
#define GPIO_INTRAWSTAT_PIN12_Msk	(0x01 << GPIO_INTRAWSTAT_PIN12_Pos)
#define GPIO_INTRAWSTAT_PIN13_Pos	13
#define GPIO_INTRAWSTAT_PIN13_Msk	(0x01 << GPIO_INTRAWSTAT_PIN13_Pos)
#define GPIO_INTRAWSTAT_PIN14_Pos	14
#define GPIO_INTRAWSTAT_PIN14_Msk	(0x01 << GPIO_INTRAWSTAT_PIN14_Pos)
#define GPIO_INTRAWSTAT_PIN15_Pos	15
#define GPIO_INTRAWSTAT_PIN15_Msk	(0x01 << GPIO_INTRAWSTAT_PIN15_Pos)

#define GPIO_INTSTAT_PIN0_Pos		0
#define GPIO_INTSTAT_PIN0_Msk		(0x01 << GPIO_INTSTAT_PIN0_Pos)
#define GPIO_INTSTAT_PIN1_Pos		1
#define GPIO_INTSTAT_PIN1_Msk		(0x01 << GPIO_INTSTAT_PIN1_Pos)
#define GPIO_INTSTAT_PIN2_Pos		2
#define GPIO_INTSTAT_PIN2_Msk		(0x01 << GPIO_INTSTAT_PIN2_Pos)
#define GPIO_INTSTAT_PIN3_Pos		3
#define GPIO_INTSTAT_PIN3_Msk		(0x01 << GPIO_INTSTAT_PIN3_Pos)
#define GPIO_INTSTAT_PIN4_Pos		4
#define GPIO_INTSTAT_PIN4_Msk		(0x01 << GPIO_INTSTAT_PIN4_Pos)
#define GPIO_INTSTAT_PIN5_Pos		5
#define GPIO_INTSTAT_PIN5_Msk		(0x01 << GPIO_INTSTAT_PIN5_Pos)
#define GPIO_INTSTAT_PIN6_Pos		6
#define GPIO_INTSTAT_PIN6_Msk		(0x01 << GPIO_INTSTAT_PIN6_Pos)
#define GPIO_INTSTAT_PIN7_Pos		7
#define GPIO_INTSTAT_PIN7_Msk		(0x01 << GPIO_INTSTAT_PIN7_Pos)
#define GPIO_INTSTAT_PIN8_Pos		8
#define GPIO_INTSTAT_PIN8_Msk		(0x01 << GPIO_INTSTAT_PIN8_Pos)
#define GPIO_INTSTAT_PIN9_Pos		9
#define GPIO_INTSTAT_PIN9_Msk		(0x01 << GPIO_INTSTAT_PIN9_Pos)
#define GPIO_INTSTAT_PIN10_Pos		10
#define GPIO_INTSTAT_PIN10_Msk		(0x01 << GPIO_INTSTAT_PIN10_Pos)
#define GPIO_INTSTAT_PIN11_Pos		11
#define GPIO_INTSTAT_PIN11_Msk		(0x01 << GPIO_INTSTAT_PIN11_Pos)
#define GPIO_INTSTAT_PIN12_Pos		12
#define GPIO_INTSTAT_PIN12_Msk		(0x01 << GPIO_INTSTAT_PIN12_Pos)
#define GPIO_INTSTAT_PIN13_Pos		13
#define GPIO_INTSTAT_PIN13_Msk		(0x01 << GPIO_INTSTAT_PIN13_Pos)
#define GPIO_INTSTAT_PIN14_Pos		14
#define GPIO_INTSTAT_PIN14_Msk		(0x01 << GPIO_INTSTAT_PIN14_Pos)
#define GPIO_INTSTAT_PIN15_Pos		15
#define GPIO_INTSTAT_PIN15_Msk		(0x01 << GPIO_INTSTAT_PIN15_Pos)

#define GPIO_INTCLR_PIN0_Pos		0
#define GPIO_INTCLR_PIN0_Msk		(0x01 << GPIO_INTCLR_PIN0_Pos)
#define GPIO_INTCLR_PIN1_Pos		1
#define GPIO_INTCLR_PIN1_Msk		(0x01 << GPIO_INTCLR_PIN1_Pos)
#define GPIO_INTCLR_PIN2_Pos		2
#define GPIO_INTCLR_PIN2_Msk		(0x01 << GPIO_INTCLR_PIN2_Pos)
#define GPIO_INTCLR_PIN3_Pos		3
#define GPIO_INTCLR_PIN3_Msk		(0x01 << GPIO_INTCLR_PIN3_Pos)
#define GPIO_INTCLR_PIN4_Pos		4
#define GPIO_INTCLR_PIN4_Msk		(0x01 << GPIO_INTCLR_PIN4_Pos)
#define GPIO_INTCLR_PIN5_Pos		5
#define GPIO_INTCLR_PIN5_Msk		(0x01 << GPIO_INTCLR_PIN5_Pos)
#define GPIO_INTCLR_PIN6_Pos		6
#define GPIO_INTCLR_PIN6_Msk		(0x01 << GPIO_INTCLR_PIN6_Pos)
#define GPIO_INTCLR_PIN7_Pos		7
#define GPIO_INTCLR_PIN7_Msk		(0x01 << GPIO_INTCLR_PIN7_Pos)
#define GPIO_INTCLR_PIN8_Pos		8
#define GPIO_INTCLR_PIN8_Msk		(0x01 << GPIO_INTCLR_PIN8_Pos)
#define GPIO_INTCLR_PIN9_Pos		9
#define GPIO_INTCLR_PIN9_Msk		(0x01 << GPIO_INTCLR_PIN9_Pos)
#define GPIO_INTCLR_PIN10_Pos		10
#define GPIO_INTCLR_PIN10_Msk		(0x01 << GPIO_INTCLR_PIN10_Pos)
#define GPIO_INTCLR_PIN11_Pos		11
#define GPIO_INTCLR_PIN11_Msk		(0x01 << GPIO_INTCLR_PIN11_Pos)
#define GPIO_INTCLR_PIN12_Pos		12
#define GPIO_INTCLR_PIN12_Msk		(0x01 << GPIO_INTCLR_PIN12_Pos)
#define GPIO_INTCLR_PIN13_Pos		13
#define GPIO_INTCLR_PIN13_Msk		(0x01 << GPIO_INTCLR_PIN13_Pos)
#define GPIO_INTCLR_PIN14_Pos		14
#define GPIO_INTCLR_PIN14_Msk		(0x01 << GPIO_INTCLR_PIN14_Pos)
#define GPIO_INTCLR_PIN15_Pos		15
#define GPIO_INTCLR_PIN15_Msk		(0x01 << GPIO_INTCLR_PIN15_Pos)




typedef struct {
	__IO uint32_t LDVAL;					//��ʱ������ֵ��24λ��ʹ�ܺ�ʱ���Ӵ���ֵ��ʼ���µݼ�����

	__I  uint32_t CVAL;					 	//��ʱ����ǰֵ��24λ��LDVAL-CVAL �ɼ������ʱʱ��

	__IO uint32_t CTRL;
} TIMR_TypeDef;


#define TIMR_CTRL_EN_Pos			0		//��λ��1����TIMR��LDVAL��ʼ���µݼ�����
#define TIMR_CTRL_EN_Msk			(0x01 << TIMR_CTRL_EN_Pos)
#define TIMR_CTRL_CLKSRC_Pos		1		//ʱ��Դ��0 �ڲ�ϵͳʱ��	1 TIMRx-1������ź�����TIMRx����	2 �ⲿ�����������
#define TIMR_CTRL_CLKSRC_Msk		(0x03 << TIMR_CTRL_CLKSRC_Pos)
#define TIMR_CTRL_OC_INILVL_Pos		3		//Output Compare Inital Level
#define TIMR_CTRL_OC_INILVL_Msk		(0x01 << TIMR_CTRL_OC_INILVL_Pos)
#define TIMR_CTRL_OC_MSKLVL_Pos		4		//����ȽϹ��ܱ�����ʱ���õ����ŵ�ƽ
#define TIMR_CTRL_OC_MSKLVL_Msk		(0x01 << TIMR_CTRL_OC_MSKLVL_Pos)
#define TIMR_CTRL_OC_MODE_Pos		5		//0 һ����ת�㣬___|--- �� ---|___    1 ������ת�㣬___|---|___ �� ---|___|---
#define TIMR_CTRL_OC_MODE_Msk		(0x01 << TIMR_CTRL_OC_MODE_Pos)




typedef struct {
	__IO uint32_t ICCR;						//Input Capture Control Register
	
		 uint32_t RESERVED[5];
	
	__IO uint32_t HALT;
	
	     uint32_t RESERVED2;
	
	__IO uint32_t HALLCR;
	
	__IO uint32_t HALLSR;
	
		 uint32_t RESERVED3[2];
	
	__IO uint32_t HALL_A;					//�����ź�A����ʱ��TIMER0ֵ��24λ
	
	__IO uint32_t HALL_B;
	
	__IO uint32_t HALL_C;
	
		 uint32_t RESERVED4[25];
		 
	__IO uint32_t IF0;						//Interrupt Flag for Timer0
	
	__IO uint32_t IF1;
	
	__IO uint32_t IF2;
	
	__IO uint32_t IF3;
	
		 uint32_t RESERVED5[4];
		 
	__IO uint32_t IE0;						//�ж�ʹ��=0��������IF�ź�
	
	__IO uint32_t IE1;
	
	__IO uint32_t IE2;
	
	__IO uint32_t IE3;
	
		 uint32_t RESERVED6[4];
		 
	__IO uint32_t IM0;						//�ж�����=1��������IF�źţ�Ҳ��������ں˴����ж�
	
	__IO uint32_t IM1;
	
	__IO uint32_t IM2;
	
	__IO uint32_t IM3;
	
		 uint32_t RESERVED7[4];
		 
	__IO uint32_t OCMAT0;					//Output Compare Match Register for Timer0������Ƚ�ģʽ��TIMRֻ��16λ��[15:0] Ϊ��һ����ת��ƥ��ֵ   [31:16] Ϊ�ڶ�����ת��ƥ��ֵ
	
	__IO uint32_t OCMAT1;
	
	__IO uint32_t OCMAT2;
	
	__IO uint32_t OCMAT3;
	
		 uint32_t RESERVED8[4];
		 
	__IO uint32_t OCEN;						//Output Compare Enabel
	
	__IO uint32_t OCMSK;					//Output Compare Maks�����κ����������ƽ��TIMRx->CTRL.OC_MSKLVL����
	
		 uint32_t RESERVED9[14];
		 
	__IO uint32_t ICVAL0H;					//Input Capture High Level Value for Timer0��24λ
	
	__IO uint32_t ICVAL0L;					//Input Capture Low Level Value for Timer0��24λ
	
	__IO uint32_t ICVAL1H;
	
	__IO uint32_t ICVAL1L;
	
	__IO uint32_t ICVAL2H;
	
	__IO uint32_t ICVAL2L;
	
	__IO uint32_t ICVAL3H;
	
	__IO uint32_t ICVAL3L;
} TIMRG_TypeDef;


#define TIMRG_ICCR_GO0_Pos			0		//Timer0�������� 1����ʼ����    0��ֹͣ����
#define TIMRG_ICCR_GO0_Msk			(0x01 << TIMRG_ICCR_GO0_Pos)
#define TIMRG_ICCR_POL0_Pos			1		//Timer0������ʼ���� 1 �����ߵ�ƽ��ʼ    0 �����͵�ƽ��ʼ
#define TIMRG_ICCR_POL0_Msk			(0x01 << TIMRG_ICCR_POL0_Pos)
#define TIMRG_ICCR_EN0_Pos			2		//Timer0��������ʹ��
#define TIMRG_ICCR_EN0_Msk			(0x01 << TIMRG_ICCR_EN0_Pos)
#define TIMRG_ICCR_GO1_Pos			4		//Timer1��������
#define TIMRG_ICCR_GO1_Msk			(0x01 << TIMRG_ICCR_GO1_Pos)
#define TIMRG_ICCR_POL1_Pos			5		//Timer1������ʼ����
#define TIMRG_ICCR_POL1_Msk			(0x01 << TIMRG_ICCR_POL1_Pos)
#define TIMRG_ICCR_EN1_Pos			6		//Timer1��������ʹ��
#define TIMRG_ICCR_EN1_Msk			(0x01 << TIMRG_ICCR_EN1_Pos)
#define TIMRG_ICCR_GO2_Pos			8
#define TIMRG_ICCR_GO2_Msk			(0x01 << TIMRG_ICCR_GO2_Pos)
#define TIMRG_ICCR_POL2_Pos			9
#define TIMRG_ICCR_POL2_Msk			(0x01 << TIMRG_ICCR_POL2_Pos)
#define TIMRG_ICCR_EN2_Pos			10
#define TIMRG_ICCR_EN2_Msk			(0x01 << TIMRG_ICCR_EN2_Pos)
#define TIMRG_ICCR_GO3_Pos			12
#define TIMRG_ICCR_GO3_Msk			(0x01 << TIMRG_ICCR_GO3_Pos)
#define TIMRG_ICCR_POL3_Pos			13
#define TIMRG_ICCR_POL3_Msk			(0x01 << TIMRG_ICCR_POL3_Pos)
#define TIMRG_ICCR_EN3_Pos			14
#define TIMRG_ICCR_EN3_Msk			(0x01 << TIMRG_ICCR_EN3_Pos)

#define TIMRG_HALT_TIMR0_Pos		0		//1 ��ͣ����
#define TIMRG_HALT_TIMR0_Msk		(0x01 << TIMRG_HALT_TIMR0_Pos)
#define TIMRG_HALT_TIMR1_Pos		1
#define TIMRG_HALT_TIMR1_Msk		(0x01 << TIMRG_HALT_TIMR1_Pos)
#define TIMRG_HALT_TIMR2_Pos		2
#define TIMRG_HALT_TIMR2_Msk		(0x01 << TIMRG_HALT_TIMR2_Pos)
#define TIMRG_HALT_TIMR3_Pos		3
#define TIMRG_HALT_TIMR3_Msk		(0x01 << TIMRG_HALT_TIMR3_Pos)

#define TIMRG_HALLCR_IEA_Pos		0		//0 �����ź�A��ֹ�ж�    1 �����ز����ж�    2 �½��ز����ж�    3 ˫���ز����ж�		
#define TIMRG_HALLCR_IEA_Msk		(0x03 << TIMRG_HALLCR_IEA_Pos)
#define TIMRG_HALLCR_IEB_Pos		2
#define TIMRG_HALLCR_IEB_Msk		(0x03 << TIMRG_HALLCR_IEB_Pos)
#define TIMRG_HALLCR_IEC_Pos		4
#define TIMRG_HALLCR_IEC_Msk		(0x03 << TIMRG_HALLCR_IEC_Pos)

#define TIMRG_HALLSR_IFA_Pos		0		//�����ź�A�жϱ�־��д1����	
#define TIMRG_HALLSR_IFA_Msk		(0x01 << TIMRG_HALLSR_IFA_Pos)
#define TIMRG_HALLSR_IFB_Pos		1
#define TIMRG_HALLSR_IFB_Msk		(0x01 << TIMRG_HALLSR_IFB_Pos)
#define TIMRG_HALLSR_IFC_Pos		2
#define TIMRG_HALLSR_IFC_Msk		(0x01 << TIMRG_HALLSR_IFC_Pos)
#define TIMRG_HALLSR_STA_Pos		3		//�����ź�A״̬��־λ
#define TIMRG_HALLSR_STA_Msk		(0x01 << TIMRG_HALLSR_STA_Pos)
#define TIMRG_HALLSR_STB_Pos		4
#define TIMRG_HALLSR_STB_Msk		(0x01 << TIMRG_HALLSR_STB_Pos)
#define TIMRG_HALLSR_STC_Pos		5
#define TIMRG_HALLSR_STC_Msk		(0x01 << TIMRG_HALLSR_STC_Pos)

#define TIMRG_IF_IF_Pos				0		//��ʱ���ݼ���0�жϣ���һ����ʱ���ڣ�д1����
#define TIMRG_IF_IF_Msk				(0x01 << TIMRG_IF_IF_Pos)
#define TIMRG_IF_OCMAT1_Pos			1		//Output Compare Match 1
#define TIMRG_IF_OCMAT1_Msk			(0x01 << TIMRG_IF_OCMAT1_Pos)
#define TIMRG_IF_OCMAT2_Pos			2		//Output Compare Match 2
#define TIMRG_IF_OCMAT2_Msk			(0x01 << TIMRG_IF_OCMAT2_Pos)
#define TIMRG_IF_ICH_Pos			8		//Input Capture High Level
#define TIMRG_IF_ICH_Msk			(0x01 << TIMRG_IF_ICH_Pos)
#define TIMRG_IF_ICL_Pos			9		//Input Capture Low Level
#define TIMRG_IF_ICL_Msk			(0x01 << TIMRG_IF_ICL_Pos)
#define TIMRG_IF_ICOV_Pos			10		//Input Capture Overflow
#define TIMRG_IF_ICOV_Msk			(0x01 << TIMRG_IF_ICOV_Pos)
#define TIMRG_IF_HALL_Pos			16
#define TIMRG_IF_HALL_Msk			(0x01 << TIMRG_IF_HALL_Pos)

#define TIMRG_IE_IF_Pos				0		//��ʱ���ݼ���0�жϣ���һ����ʱ����
#define TIMRG_IE_IF_Msk				(0x01 << TIMRG_IE_IF_Pos)
#define TIMRG_IE_OCMAT1_Pos			1		//Output Compare Match 1
#define TIMRG_IE_OCMAT1_Msk			(0x01 << TIMRG_IE_OCMAT1_Pos)
#define TIMRG_IE_OCMAT2_Pos			2		//Output Compare Match 2
#define TIMRG_IE_OCMAT2_Msk			(0x01 << TIMRG_IE_OCMAT2_Pos)
#define TIMRG_IE_ICH_Pos			8		//Input Capture High Level
#define TIMRG_IE_ICH_Msk			(0x01 << TIMRG_IE_ICH_Pos)
#define TIMRG_IE_ICL_Pos			9		//Input Capture Low Level
#define TIMRG_IE_ICL_Msk			(0x01 << TIMRG_IE_ICL_Pos)
#define TIMRG_IE_ICOV_Pos			10		//Input Capture Overflow
#define TIMRG_IE_ICOV_Msk			(0x01 << TIMRG_IE_ICOV_Pos)
#define TIMRG_IE_HALL_Pos			16
#define TIMRG_IE_HALL_Msk			(0x01 << TIMRG_IE_HALL_Pos)

#define TIMRG_IM_IF_Pos				0		//��ʱ���ݼ���0�жϣ���һ����ʱ����
#define TIMRG_IM_IF_Msk				(0x01 << TIMRG_IM_IF_Pos)
#define TIMRG_IM_OCMAT1_Pos			1		//Output Compare Match 1
#define TIMRG_IM_OCMAT1_Msk			(0x01 << TIMRG_IM_OCMAT1_Pos)
#define TIMRG_IM_OCMAT2_Pos			2		//Output Compare Match 2
#define TIMRG_IM_OCMAT2_Msk			(0x01 << TIMRG_IM_OCMAT2_Pos)
#define TIMRG_IM_ICH_Pos			8		//Input Capture High Level
#define TIMRG_IM_ICH_Msk			(0x01 << TIMRG_IM_ICH_Pos)
#define TIMRG_IM_ICL_Pos			9		//Input Capture Low Level
#define TIMRG_IM_ICL_Msk			(0x01 << TIMRG_IM_ICL_Pos)
#define TIMRG_IM_ICOV_Pos			10		//Input Capture Overflow
#define TIMRG_IM_ICOV_Msk			(0x01 << TIMRG_IM_ICOV_Pos)
#define TIMRG_IM_HALL_Pos			16
#define TIMRG_IM_HALL_Msk			(0x01 << TIMRG_IM_HALL_Pos)

#define TIMRG_OCEN_TIMR0_Pos		0
#define TIMRG_OCEN_TIMR0_Msk		(0x01 << TIMRG_OCEN_TIMR0_Pos)
#define TIMRG_OCEN_TIMR1_Pos		1
#define TIMRG_OCEN_TIMR1_Msk		(0x01 << TIMRG_OCEN_TIMR1_Pos)
#define TIMRG_OCEN_TIMR2_Pos		2
#define TIMRG_OCEN_TIMR2_Msk		(0x01 << TIMRG_OCEN_TIMR2_Pos)
#define TIMRG_OCEN_TIMR3_Pos		3
#define TIMRG_OCEN_TIMR3_Msk		(0x01 << TIMRG_OCEN_TIMR3_Pos)

#define TIMRG_OCMSK_TIMR0_Pos		0
#define TIMRG_OCMSK_TIMR0_Msk		(0x01 << TIMRG_OCMSK_TIMR0_Pos)
#define TIMRG_OCMSK_TIMR1_Pos		1
#define TIMRG_OCMSK_TIMR1_Msk		(0x01 << TIMRG_OCMSK_TIMR1_Pos)
#define TIMRG_OCMSK_TIMR2_Pos		2
#define TIMRG_OCMSK_TIMR2_Msk		(0x01 << TIMRG_OCMSK_TIMR2_Pos)
#define TIMRG_OCMSK_TIMR3_Pos		3
#define TIMRG_OCMSK_TIMR3_Msk		(0x01 << TIMRG_OCMSK_TIMR3_Pos)




typedef struct {
	__IO uint32_t DATA;
	
	__IO uint32_t CTRL;
	
	__IO uint32_t BAUD;
	
	__IO uint32_t FIFO;
} UART_TypeDef;


#define UART_DATA_DATA_Pos			0
#define UART_DATA_DATA_Msk			(0x1FF << UART_DATA_DATA_Pos)
#define UART_DATA_VALID_Pos			9		//��DATA�ֶ�����Ч�Ľ�������ʱ����λӲ����1����ȡ���ݺ��Զ�����
#define UART_DATA_VALID_Msk			(0x01 << UART_DATA_VALID_Pos)
#define UART_DATA_PAERR_Pos			10		//Parity Error
#define UART_DATA_PAERR_Msk			(0x01 << UART_DATA_PAERR_Pos)

#define UART_CTRL_TXIDLE_Pos		0		//TX IDLE: 0 ���ڷ�������	1 ����״̬��û�����ݷ���
#define UART_CTRL_TXIDLE_Msk		(0x01 << UART_CTRL_TXIDLE_Pos)
#define UART_CTRL_TXF_Pos		    1		//TX FIFO Full
#define UART_CTRL_TXF_Msk		    (0x01 << UART_CTRL_TXF_Pos)
#define UART_CTRL_TXIE_Pos			2		//TX �ж�ʹ��: 1 TX FF �����������趨����ʱ�����ж�
#define UART_CTRL_TXIE_Msk			(0x01 << UART_CTRL_TXIE_Pos)
#define UART_CTRL_RXNE_Pos			3		//RX FIFO Not Empty
#define UART_CTRL_RXNE_Msk			(0x01 << UART_CTRL_RXNE_Pos)
#define UART_CTRL_RXIE_Pos			4		//RX �ж�ʹ��: 1 RX FF �����ݴﵽ�趨����ʱ�����ж�
#define UART_CTRL_RXIE_Msk			(0x01 << UART_CTRL_RXIE_Pos)
#define UART_CTRL_RXOV_Pos			5		//RX FIFO Overflow��д1����
#define UART_CTRL_RXOV_Msk			(0x01 << UART_CTRL_RXOV_Pos)
#define UART_CTRL_EN_Pos			9
#define UART_CTRL_EN_Msk			(0x01 << UART_CTRL_EN_Pos)
#define UART_CTRL_LOOP_Pos			10
#define UART_CTRL_LOOP_Msk			(0x01 << UART_CTRL_LOOP_Pos)
#define UART_CTRL_BAUDEN_Pos		13		//����д1
#define UART_CTRL_BAUDEN_Msk		(0x01 << UART_CTRL_BAUDEN_Pos)
#define UART_CTRL_TOIE_Pos			14		//TimeOut �ж�ʹ�ܣ����յ��ϸ��ַ��󣬳��� TOTIME/BAUDRAUD ��û�н��յ��µ�����
#define UART_CTRL_TOIE_Msk			(0x01 << UART_CTRL_TOIE_Pos)
#define UART_CTRL_BRKDET_Pos		15		//LIN Break Detect����⵽LIN Break����RX���ϼ�⵽����11λ�͵�ƽ
#define UART_CTRL_BRKDET_Msk		(0x01 << UART_CTRL_BRKDET_Pos)
#define UART_CTRL_BRKIE_Pos			16		//LIN Break Detect �ж�ʹ��
#define UART_CTRL_BRKIE_Msk			(0x01 << UART_CTRL_BRKIE_Pos)
#define UART_CTRL_GENBRK_Pos		17		//Generate LIN Break������LIN Break
#define UART_CTRL_GENBRK_Msk		(0x01 << UART_CTRL_GENBRK_Pos)
#define UART_CTRL_DATA9b_Pos		18		//1 9λ����λ    0 8λ����λ
#define UART_CTRL_DATA9b_Msk		(0x01 << UART_CTRL_DATA9b_Pos)
#define UART_CTRL_PARITY_Pos		19		//0 ����żУ��    1 ��У��    3 żУ��    5 �̶�Ϊ1    7 �̶�Ϊ0
#define UART_CTRL_PARITY_Msk		(0x07 << UART_CTRL_PARITY_Pos)
#define UART_CTRL_STOP2b_Pos		22		//1 2λֹͣλ    0 1λֹͣλ
#define UART_CTRL_STOP2b_Msk		(0x03 << UART_CTRL_STOP2b_Pos)
#define UART_CTRL_TOTIME_Pos		24		//TimeOut ʱ�� = TOTIME/(BAUDRAUD/10) ��
//#define UART_CTRL_TOTIME_Msk		(0xFF << UART_CTRL_TOTIME_Pos)	���������棺 integer operation result is out of range
#define UART_CTRL_TOTIME_Msk		((uint32_t)0xFF << UART_CTRL_TOTIME_Pos)

#define UART_BAUD_BAUD_Pos			0		//���ڲ����� = SYS_Freq/16/BAUD - 1
#define UART_BAUD_BAUD_Msk			(0x3FFF << UART_BAUD_BAUD_Pos)
#define UART_BAUD_TXD_Pos			14		//ͨ����λ��ֱ�Ӷ�ȡ����TXD�����ϵĵ�ƽ
#define UART_BAUD_TXD_Msk			(0x01 << UART_BAUD_TXD_Pos)
#define UART_BAUD_RXDn_Pos			15		//ͨ����λ��ֱ�Ӷ�ȡ����RXD�����ϵĵ�ƽ��ȡ��ֵ
#define UART_BAUD_RXDn_Msk			(0x01 << UART_BAUD_RXDn_Pos)
#define UART_BAUD_RXTOIF_Pos		16		//����&��ʱ���жϱ�־ = RXIF | TOIF
#define UART_BAUD_RXTOIF_Msk		(0x01 << UART_BAUD_RXTOIF_Pos)
#define UART_BAUD_TXIF_Pos			17		//�����жϱ�־ = TXTHRF & TXIE
#define UART_BAUD_TXIF_Msk			(0x01 << UART_BAUD_TXIF_Pos)
#define UART_BAUD_BRKIF_Pos			18		//LIN Break Detect �жϱ�־����⵽LIN Breakʱ��BRKIE=1����λ��Ӳ����λ
#define UART_BAUD_BRKIF_Msk			(0x01 << UART_BAUD_BRKIF_Pos)
#define UART_BAUD_RXTHRF_Pos		19		//RX FIFO Threshold Flag��RX FIFO�����ݴﵽ�趨������RXLVL >  RXTHR��ʱӲ����1
#define UART_BAUD_RXTHRF_Msk		(0x01 << UART_BAUD_RXTHRF_Pos)
#define UART_BAUD_TXTHRF_Pos		20		//TX FIFO Threshold Flag��TX FIFO�����������趨������TXLVL <= TXTHR��ʱӲ����1
#define UART_BAUD_TXTHRF_Msk		(0x01 << UART_BAUD_TXTHRF_Pos)
#define UART_BAUD_TOIF_Pos			21		//TimeOut �жϱ�־������ TOTIME/BAUDRAUD ��û�н��յ��µ�����ʱ��TOIE=1����λ��Ӳ����λ
#define UART_BAUD_TOIF_Msk			(0x01 << UART_BAUD_TOIF_Pos)
#define UART_BAUD_RXIF_Pos			22		//�����жϱ�־ = RXTHRF & RXIE
#define UART_BAUD_RXIF_Msk			(0x01 << UART_BAUD_RXIF_Pos)

#define UART_FIFO_RXLVL_Pos			0		//RX FIFO Level��RX FIFO ���ַ�����
#define UART_FIFO_RXLVL_Msk			(0xFF << UART_FIFO_RXLVL_Pos)
#define UART_FIFO_TXLVL_Pos			8		//TX FIFO Level��TX FIFO ���ַ�����
#define UART_FIFO_TXLVL_Msk			(0xFF << UART_FIFO_TXLVL_Pos)
#define UART_FIFO_RXTHR_Pos			16		//RX FIFO Threshold��RX�жϴ������ޣ��ж�ʹ��ʱ RXLVL >  RXTHR ����RX�ж�
#define UART_FIFO_RXTHR_Msk			(0xFF << UART_FIFO_RXTHR_Pos)
#define UART_FIFO_TXTHR_Pos			24		//TX FIFO Threshold��TX�жϴ������ޣ��ж�ʹ��ʱ TXLVL <= TXTHR ����TX�ж�
//#define UART_FIFO_TXTHR_Msk			(0xFF << UART_FIFO_TXTHR_Pos)	���������棺 integer operation result is out of range
#define UART_FIFO_TXTHR_Msk			((uint32_t)0xFF << UART_FIFO_TXTHR_Pos)




typedef struct {
	__IO uint32_t CTRL;

	__IO uint32_t DATA;

	__IO uint32_t STAT;

	__IO uint32_t IE;

	__IO uint32_t IF;
} SPI_TypeDef;


#define SPI_CTRL_CLKDIV_Pos			0		//Clock Divider, SPI����ʱ�� = SYS_Freq/pow(2, CLKDIV+2)
#define SPI_CTRL_CLKDIV_Msk			(0x07 << SPI_CTRL_CLKDIV_Pos)
#define SPI_CTRL_EN_Pos				3
#define SPI_CTRL_EN_Msk				(0x01 << SPI_CTRL_EN_Pos)
#define SPI_CTRL_DSS_Pos			4		//Data Size Select, ȡֵ3--15����ʾ4--16λ
#define SPI_CTRL_DSS_Msk			(0x0F << SPI_CTRL_DSS_Pos)
#define SPI_CTRL_CPHA_Pos			8		//0 ��SCLK�ĵ�һ�������ز�������	1 ��SCLK�ĵڶ��������ز�������
#define SPI_CTRL_CPHA_Msk			(0x01 << SPI_CTRL_CPHA_Pos)
#define SPI_CTRL_CPOL_Pos			9		//0 ����״̬��SCLKΪ�͵�ƽ		  1 ����״̬��SCLKΪ�ߵ�ƽ
#define SPI_CTRL_CPOL_Msk			(0x01 << SPI_CTRL_CPOL_Pos)
#define SPI_CTRL_FFS_Pos			10		//Frame Format Select, 0 SPI	1 TI SSI	2 SPI	3 SPI
#define SPI_CTRL_FFS_Msk			(0x03 << SPI_CTRL_FFS_Pos)
#define SPI_CTRL_MSTR_Pos			12		//Master, 1 ��ģʽ	0 ��ģʽ
#define SPI_CTRL_MSTR_Msk			(0x01 << SPI_CTRL_MSTR_Pos)
#define SPI_CTRL_FAST_Pos			13		//1 SPI����ʱ�� = SYS_Freq/2    0 SPI����ʱ����SPI->CTRL.CLKDIV����
#define SPI_CTRL_FAST_Msk			(0x01 << SPI_CTRL_FAST_Pos)
#define SPI_CTRL_DMAEN_Pos			14		//1 ͨ��DMA CH0��дFIFO    0 ͨ��MCU��дFIFO
#define SPI_CTRL_DMAEN_Msk			(0x01 << SPI_CTRL_DMAEN_Pos)
#define SPI_CTRL_FILTE_Pos			16		//1 ��SPI�����źŽ���ȥ������    0 ��SPI�����źŲ�����ȥ������
#define SPI_CTRL_FILTE_Msk			(0x01 << SPI_CTRL_FILTE_Pos)
#define SPI_CTRL_SSN_H_Pos			17		//0 ���������SSNʼ��Ϊ0    	 1 ���������ÿ�ַ�֮��ὫSSN���߰��SCLK����
#define SPI_CTRL_SSN_H_Msk			(0x01 << SPI_CTRL_SSN_H_Pos)

#define SPI_STAT_TC_Pos				0		//Transmit Complete��ÿ�������һ������֡��Ӳ����1�����д1����
#define SPI_STAT_TC_Msk				(0x01 << SPI_STAT_TC_Pos)
#define SPI_STAT_TFE_Pos			1		//����FIFO Empty
#define SPI_STAT_TFE_Msk			(0x01 << SPI_STAT_TFE_Pos)
#define SPI_STAT_TFNF_Pos			2		//����FIFO Not Full
#define SPI_STAT_TFNF_Msk			(0x01 << SPI_STAT_TFNF_Pos)
#define SPI_STAT_RFNE_Pos			3		//����FIFO Not Empty
#define SPI_STAT_RFNE_Msk			(0x01 << SPI_STAT_RFNE_Pos)
#define SPI_STAT_RFF_Pos			4		//����FIFO Full
#define SPI_STAT_RFF_Msk			(0x01 << SPI_STAT_RFF_Pos)
#define SPI_STAT_RFOVF_Pos			5		//����FIFO Overflow
#define SPI_STAT_RFOVF_Msk			(0x01 << SPI_STAT_RFOVF_Pos)
#define SPI_STAT_TFLVL_Pos			6		//����FIFO�����ݸ����� 0 TFNF=0ʱ��ʾFIFO����8�����ݣ�TFNF=1ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_TFLVL_Msk			(0x07 << SPI_STAT_TFLVL_Pos)
#define SPI_STAT_RFLVL_Pos			9		//����FIFO�����ݸ����� 0 RFF=1ʱ��ʾFIFO����8�����ݣ� RFF=0ʱ��ʾFIFO����0������	1--7 FIFO����1--7������
#define SPI_STAT_RFLVL_Msk			(0x07 << SPI_STAT_RFLVL_Pos)

#define SPI_IE_RFOVF_Pos			0
#define SPI_IE_RFOVF_Msk			(0x01 << SPI_IE_RFOVF_Pos)
#define SPI_IE_RFF_Pos				1
#define SPI_IE_RFF_Msk				(0x01 << SPI_IE_RFF_Pos)
#define SPI_IE_RFHF_Pos				2
#define SPI_IE_RFHF_Msk				(0x01 << SPI_IE_RFHF_Pos)
#define SPI_IE_TFE_Pos				3
#define SPI_IE_TFE_Msk				(0x01 << SPI_IE_TFE_Pos)
#define SPI_IE_TFHF_Pos				4
#define SPI_IE_TFHF_Msk				(0x01 << SPI_IE_TFHF_Pos)

#define SPI_IF_RFOVF_Pos			0		//д1����
#define SPI_IF_RFOVF_Msk			(0x01 << SPI_IF_RFOVF_Pos)
#define SPI_IF_RFF_Pos				1
#define SPI_IF_RFF_Msk				(0x01 << SPI_IF_RFF_Pos)
#define SPI_IF_RFHF_Pos				2
#define SPI_IF_RFHF_Msk				(0x01 << SPI_IF_RFHF_Pos)
#define SPI_IF_TFE_Pos				3
#define SPI_IF_TFE_Msk				(0x01 << SPI_IF_TFE_Pos)
#define SPI_IF_TFHF_Pos				4
#define SPI_IF_TFHF_Msk				(0x01 << SPI_IF_TFHF_Pos)




typedef struct {
	__IO uint32_t CLKDIV;				   	//[15:0] �뽫�ڲ�����Ƶ�ʷֵ�SCLƵ�ʵ�5������CLKDIV = SYS_Freq/5/SCL_Freq - 1

	__IO uint32_t CTRL;

	__IO uint32_t MSTDAT;

	__IO uint32_t MSTCMD;
	
	__IO uint32_t SLVCR;
	
	__IO uint32_t SLVIF;
	
	__IO uint32_t SLVTX;
	
	__IO uint32_t SLVRX;
} I2C_TypeDef;


#define I2C_CTRL_MSTIE_Pos			6
#define I2C_CTRL_MSTIE_Msk			(0x01 << I2C_CTRL_MSTIE_Pos)
#define I2C_CTRL_EN_Pos				7
#define I2C_CTRL_EN_Msk				(0x01 << I2C_CTRL_EN_Pos)

#define I2C_MSTCMD_IF_Pos			0		//1 �еȴ�������жϣ�д1����	����������´�λӲ����λ��1��һ���ֽڴ������  2�����߷���Ȩ��ʧ
#define I2C_MSTCMD_IF_Msk			(0x01 << I2C_MSTCMD_IF_Pos)
#define I2C_MSTCMD_TIP_Pos			1		//Transmission In Process
#define I2C_MSTCMD_TIP_Msk			(0x01 << I2C_MSTCMD_TIP_Pos)
#define I2C_MSTCMD_ACK_Pos			3		//����ģʽ�£�0 ���Ͷ˷���ACK	1 ���Ͷ˷���NACK
#define I2C_MSTCMD_ACK_Msk			(0x01 << I2C_MSTCMD_ACK_Pos)
#define I2C_MSTCMD_WR_Pos			4		//	  ��Slaveд����ʱ������һλд1���Զ�����
#define I2C_MSTCMD_WR_Msk			(0x01 << I2C_MSTCMD_WR_Pos)
#define I2C_MSTCMD_RD_Pos			5		//д����Slave������ʱ������һλд1���Զ�����	������I2Cģ��ʧȥ���ߵķ���ȨʱӲ����1
#define I2C_MSTCMD_RD_Msk			(0x01 << I2C_MSTCMD_RD_Pos)
#define I2C_MSTCMD_BUSY_Pos			6		//��������⵽START֮����һλ��1������⵽STOP֮����һλ��0
#define I2C_MSTCMD_BUSY_Msk			(0x01 << I2C_MSTCMD_BUSY_Pos)
#define I2C_MSTCMD_STO_Pos			6		//д������STOP���Զ�����
#define I2C_MSTCMD_STO_Msk			(0x01 << I2C_MSTCMD_STO_Pos)
#define I2C_MSTCMD_RXACK_Pos		7		//�������յ���Slave��ACKλ��0 �յ�ACK	1 �յ�NACK
#define I2C_MSTCMD_RXACK_Msk		(0x01 << I2C_MSTCMD_RXACK_Pos)
#define I2C_MSTCMD_STA_Pos			7		//д������START���Զ�����
#define I2C_MSTCMD_STA_Msk			(0x01 << I2C_MSTCMD_STA_Pos)

#define I2C_SLVCR_IM_RXEND_Pos		0		//��������жϽ�ֹ
#define I2C_SLVCR_IM_RXEND_Msk		(0x01 << I2C_SLVCR_IM_RXEND_Pos)
#define I2C_SLVCR_IM_TXEND_Pos		1		//��������жϽ�ֹ
#define I2C_SLVCR_IM_TXEND_Msk		(0x01 << I2C_SLVCR_IM_TXEND_Pos)
#define I2C_SLVCR_IM_STADET_Pos		2		//��⵽��ʼ�жϽ�ֹ
#define I2C_SLVCR_IM_STADET_Msk		(0x01 << I2C_SLVCR_IM_STADET_Pos)
#define I2C_SLVCR_IM_STODET_Pos		3		//��⵽ֹͣ�жϽ�ֹ
#define I2C_SLVCR_IM_STODET_Msk		(0x01 << I2C_SLVCR_IM_STODET_Pos)
#define I2C_SLVCR_IM_RDREQ_Pos		4		//���յ��������жϽ�ֹ
#define I2C_SLVCR_IM_RDREQ_Msk		(0x01 << I2C_SLVCR_IM_RDREQ_Pos)
#define I2C_SLVCR_IM_WRREQ_Pos		5		//���յ�д�����жϽ�ֹ
#define I2C_SLVCR_IM_WRREQ_Msk		(0x01 << I2C_SLVCR_IM_WRREQ_Pos)
#define I2C_SLVCR_ADDR7b_Pos		16		//1 7λ��ַģʽ    0 10λ��ַģʽ
#define I2C_SLVCR_ADDR7b_Msk		(0x01 << I2C_SLVCR_ADDR7b_Pos)
#define I2C_SLVCR_ACK_Pos			17		//1 Ӧ��ACK    0 Ӧ��NACK
#define I2C_SLVCR_ACK_Msk			(0x01 << I2C_SLVCR_ACK_Pos)
#define I2C_SLVCR_SLAVE_Pos			18		//1 �ӻ�ģʽ   0 ����ģʽ
#define I2C_SLVCR_SLAVE_Msk			(0x01 << I2C_SLVCR_SLAVE_Pos)
#define I2C_SLVCR_DEBOUNCE_Pos		19		//ȥ����ʹ��
#define I2C_SLVCR_DEBOUNCE_Msk		(0x01 << I2C_SLVCR_DEBOUNCE_Pos)
#define I2C_SLVCR_ADDR_Pos			20		//�ӻ���ַ
#define I2C_SLVCR_ADDR_Msk			(0x3FF << I2C_SLVCR_ADDR_Pos)

#define I2C_SLVIF_RXEND_Pos			0		//��������жϱ�־��д1����
#define I2C_SLVIF_RXEND_Msk			(0x01 << I2C_SLVIF_RXEND_Pos)
#define I2C_SLVIF_TXEND_Pos			1		//��������жϱ�־��д1����
#define I2C_SLVIF_TXEND_Msk			(0x01 << I2C_SLVIF_TXEND_Pos)
#define I2C_SLVIF_STADET_Pos		2		//��⵽��ʼ�жϱ�־��д1����
#define I2C_SLVIF_STADET_Msk		(0x01 << I2C_SLVIF_STADET_Pos)
#define I2C_SLVIF_STODET_Pos		3		//��⵽ֹͣ�жϱ�־��д1����
#define I2C_SLVIF_STODET_Msk		(0x01 << I2C_SLVIF_STODET_Pos)
#define I2C_SLVIF_RDREQ_Pos			4		//���յ��������жϱ�־
#define I2C_SLVIF_RDREQ_Msk			(0x01 << I2C_SLVIF_RDREQ_Pos)
#define I2C_SLVIF_WRREQ_Pos			5		//���յ�д�����жϱ�־
#define I2C_SLVIF_WRREQ_Msk			(0x01 << I2C_SLVIF_WRREQ_Pos)
#define I2C_SLVIF_ACTIVE_Pos		6		//slave ��Ч
#define I2C_SLVIF_ACTIVE_Msk		(0x01 << I2C_SLVIF_ACTIVE_Pos)




typedef struct {
	__IO uint32_t CTRL;
	
	__IO uint32_t START;
	
	__IO uint32_t IE;
	
	__IO uint32_t IF;
	
	struct {
		__IO uint32_t STAT;
		
		__IO uint32_t DATA;
		
			 uint32_t RESERVED[2];
	} CH[8];
	
	__IO uint32_t FFSTAT;				   //FIFO STAT
    
	__IO uint32_t FFDATA;				   //FIFO DATA
	
	     uint32_t RESERVED[2];
	
	__IO uint32_t CTRL1;
	
	__IO uint32_t CTRL2;
	
		 uint32_t RESERVED2[18];
	
	__IO uint32_t DEBUG;
	
	__IO uint32_t CALIBSET;
	
	__IO uint32_t CALIBEN;
} ADC_TypeDef;


#define ADC_CTRL_CH0_Pos			0		//ͨ��ѡ��
#define ADC_CTRL_CH0_Msk			(0x01 << ADC_CTRL_CH0_Pos)
#define ADC_CTRL_CH1_Pos			1
#define ADC_CTRL_CH1_Msk			(0x01 << ADC_CTRL_CH1_Pos)
#define ADC_CTRL_CH2_Pos			2
#define ADC_CTRL_CH2_Msk			(0x01 << ADC_CTRL_CH2_Pos)
#define ADC_CTRL_CH3_Pos			3
#define ADC_CTRL_CH3_Msk			(0x01 << ADC_CTRL_CH3_Pos)
#define ADC_CTRL_CH4_Pos			4
#define ADC_CTRL_CH4_Msk			(0x01 << ADC_CTRL_CH4_Pos)
#define ADC_CTRL_CH5_Pos			5
#define ADC_CTRL_CH5_Msk			(0x01 << ADC_CTRL_CH5_Pos)
#define ADC_CTRL_CH6_Pos			6
#define ADC_CTRL_CH6_Msk			(0x01 << ADC_CTRL_CH6_Pos)
#define ADC_CTRL_CH7_Pos			7
#define ADC_CTRL_CH7_Msk			(0x01 << ADC_CTRL_CH7_Pos)
#define ADC_CTRL_AVG_Pos			8		//0 1�β���	  1 2�β���ȡƽ��ֵ	  3 4�β���ȡƽ��ֵ	  7 8�β���ȡƽ��ֵ	  15 16�β���ȡƽ��ֵ
#define ADC_CTRL_AVG_Msk			(0x0F << ADC_CTRL_AVG_Pos)
#define ADC_CTRL_EN_Pos				12
#define ADC_CTRL_EN_Msk				(0x01 << ADC_CTRL_EN_Pos)
#define ADC_CTRL_CONT_Pos			13		//Continuous conversion��ֻ���������ģʽ����Ч��0 ����ת����ת����ɺ�STARTλ�Զ����ֹͣת��
#define ADC_CTRL_CONT_Msk			(0x01 << ADC_CTRL_CONT_Pos)							//   1 ����ת����������һֱ������ת����ֱ��������STARTλ
#define ADC_CTRL_TRIG_Pos			14		//ת��������ʽ��0 �������ת��	  1 PWM����	  2 TIMR2����	 3 TIMR3����
#define ADC_CTRL_TRIG_Msk			(0x03 << ADC_CTRL_TRIG_Pos)
#define ADC_CTRL_RST_Pos			16
#define ADC_CTRL_RST_Msk			(0x01 << ADC_CTRL_RST_Pos)
#define ADC_CTRL_DMAEN_Pos			17		//ֻ����ֻѡ��һ��ͨ����ʱ��ʹ��DMA���ܣ���RES2FF������1��DMA CH1 ͨ����ȡFFDATA�Ĵ�����ȡת�����
#define ADC_CTRL_DMAEN_Msk			(0x01 << ADC_CTRL_DMAEN_Pos)
#define ADC_CTRL_RES2FF_Pos			18		//Result to FIFO	1 ת���������FIFO	 0 ת�����������ӦCH��DATA�Ĵ���
#define ADC_CTRL_RES2FF_Msk			(0x01 << ADC_CTRL_RES2FF_Pos)

#define ADC_START_GO_Pos			0		//�������ģʽ�£�д1����ADC������ת�����ڵ���ģʽ��ת����ɺ�Ӳ���Զ����㣬��ɨ��ģʽ�±������д0ֹͣADCת��
#define ADC_START_GO_Msk			(0x01 << ADC_START_GO_Pos)
#define ADC_START_BUSY_Pos			4
#define ADC_START_BUSY_Msk			(0x01 << ADC_START_BUSY_Pos)

#define ADC_IE_CH0EOC_Pos			0		//End Of Convertion
#define ADC_IE_CH0EOC_Msk			(0x01 << ADC_IE_CH0EOC_Pos)
#define ADC_IE_CH0OVF_Pos			1		//Overflow
#define ADC_IE_CH0OVF_Msk			(0x01 << ADC_IE_CH0OVF_Pos)
#define ADC_IE_CH1EOC_Pos			2
#define ADC_IE_CH1EOC_Msk			(0x01 << ADC_IE_CH1EOC_Pos)
#define ADC_IE_CH1OVF_Pos			3
#define ADC_IE_CH1OVF_Msk			(0x01 << ADC_IE_CH1OVF_Pos)
#define ADC_IE_CH2EOC_Pos			4
#define ADC_IE_CH2EOC_Msk			(0x01 << ADC_IE_CH2EOC_Pos)
#define ADC_IE_CH2OVF_Pos			5
#define ADC_IE_CH2OVF_Msk			(0x01 << ADC_IE_CH2OVF_Pos)
#define ADC_IE_CH3EOC_Pos			6
#define ADC_IE_CH3EOC_Msk			(0x01 << ADC_IE_CH3EOC_Pos)
#define ADC_IE_CH3OVF_Pos			7
#define ADC_IE_CH3OVF_Msk			(0x01 << ADC_IE_CH3OVF_Pos)
#define ADC_IE_CH4EOC_Pos			8
#define ADC_IE_CH4EOC_Msk			(0x01 << ADC_IE_CH4EOC_Pos)
#define ADC_IE_CH4OVF_Pos			9
#define ADC_IE_CH4OVF_Msk			(0x01 << ADC_IE_CH4OVF_Pos)
#define ADC_IE_CH5EOC_Pos			10
#define ADC_IE_CH5EOC_Msk			(0x01 << ADC_IE_CH5EOC_Pos)
#define ADC_IE_CH5OVF_Pos			11
#define ADC_IE_CH5OVF_Msk			(0x01 << ADC_IE_CH5OVF_Pos)
#define ADC_IE_CH6EOC_Pos			12
#define ADC_IE_CH6EOC_Msk			(0x01 << ADC_IE_CH6EOC_Pos)
#define ADC_IE_CH6OVF_Pos			13
#define ADC_IE_CH6OVF_Msk			(0x01 << ADC_IE_CH6OVF_Pos)
#define ADC_IE_CH7EOC_Pos			14
#define ADC_IE_CH7EOC_Msk			(0x01 << ADC_IE_CH7EOC_Pos)
#define ADC_IE_CH7OVF_Pos			15
#define ADC_IE_CH7OVF_Msk			(0x01 << ADC_IE_CH7OVF_Pos)
#define ADC_IE_FIFOOV_Pos			16
#define ADC_IE_FIFOOV_Msk			(0x01 << ADC_IE_FIFOOV_Pos)
#define ADC_IE_FIFOHF_Pos			17
#define ADC_IE_FIFOHF_Msk			(0x01 << ADC_IE_FIFOHF_Pos)
#define ADC_IE_FIFOF_Pos			18
#define ADC_IE_FIFOF_Msk			(0x01 << ADC_IE_FIFOF_Pos)

#define ADC_IF_CH0EOC_Pos			0		//д1����
#define ADC_IF_CH0EOC_Msk			(0x01 << ADC_IF_CH0EOC_Pos)
#define ADC_IF_CH0OVF_Pos			1
#define ADC_IF_CH0OVF_Msk			(0x01 << ADC_IF_CH0OVF_Pos)
#define ADC_IF_CH1EOC_Pos			2
#define ADC_IF_CH1EOC_Msk			(0x01 << ADC_IF_CH1EOC_Pos)
#define ADC_IF_CH1OVF_Pos			3
#define ADC_IF_CH1OVF_Msk			(0x01 << ADC_IF_CH1OVF_Pos)
#define ADC_IF_CH2EOC_Pos			4
#define ADC_IF_CH2EOC_Msk			(0x01 << ADC_IF_CH2EOC_Pos)
#define ADC_IF_CH2OVF_Pos			5
#define ADC_IF_CH2OVF_Msk			(0x01 << ADC_IF_CH2OVF_Pos)
#define ADC_IF_CH3EOC_Pos			6
#define ADC_IF_CH3EOC_Msk			(0x01 << ADC_IF_CH3EOC_Pos)
#define ADC_IF_CH3OVF_Pos			7
#define ADC_IF_CH3OVF_Msk			(0x01 << ADC_IF_CH3OVF_Pos)
#define ADC_IF_CH4EOC_Pos			8
#define ADC_IF_CH4EOC_Msk			(0x01 << ADC_IF_CH4EOC_Pos)
#define ADC_IF_CH4OVF_Pos			9
#define ADC_IF_CH4OVF_Msk			(0x01 << ADC_IF_CH4OVF_Pos)
#define ADC_IF_CH5EOC_Pos			10
#define ADC_IF_CH5EOC_Msk			(0x01 << ADC_IF_CH5EOC_Pos)
#define ADC_IF_CH5OVF_Pos			11
#define ADC_IF_CH5OVF_Msk			(0x01 << ADC_IF_CH5OVF_Pos)
#define ADC_IF_CH6EOC_Pos			12
#define ADC_IF_CH6EOC_Msk			(0x01 << ADC_IF_CH6EOC_Pos)
#define ADC_IF_CH6OVF_Pos			13
#define ADC_IF_CH6OVF_Msk			(0x01 << ADC_IF_CH6OVF_Pos)
#define ADC_IF_CH7EOC_Pos			14
#define ADC_IF_CH7EOC_Msk			(0x01 << ADC_IF_CH7EOC_Pos)
#define ADC_IF_CH7OVF_Pos			15
#define ADC_IF_CH7OVF_Msk			(0x01 << ADC_IF_CH7OVF_Pos)
#define ADC_IF_FIFOOV_Pos			16
#define ADC_IF_FIFOOV_Msk			(0x01 << ADC_IF_FIFOOV_Pos)
#define ADC_IF_FIFOHF_Pos			17
#define ADC_IF_FIFOHF_Msk			(0x01 << ADC_IF_FIFOHF_Pos)
#define ADC_IF_FIFOF_Pos			18
#define ADC_IF_FIFOF_Msk			(0x01 << ADC_IF_FIFOF_Pos)

#define ADC_STAT_EOC_Pos			0		//д1����
#define ADC_STAT_EOC_Msk			(0x01 << ADC_STAT_EOC_Pos)
#define ADC_STAT_OVF_Pos			1		//�����ݼĴ������
#define ADC_STAT_OVF_Msk			(0x01 << ADC_STAT_OVF_Pos)

#define ADC_DATA_VALUE_Pos			0		//������ٴ�ת�������ݻḲ�Ǿ�����
#define ADC_DATA_VALUE_Msk			(0xFFF << ADC_DATA_VALUE_Pos)
#define ADC_DATA_CHNUM_Pos			12
#define ADC_DATA_CHNUM_Msk			(0x07 << ADC_DATA_CHNUM_Pos)

#define ADC_FFSTAT_OVF_Pos			0
#define ADC_FFSTAT_OVF_Msk			(0x01 << ADC_FFSTAT_OVF_Pos)
#define ADC_FFSTAT_HFULL_Pos		1
#define ADC_FFSTAT_HFULL_Msk		(0x01 << ADC_FFSTAT_HFULL_Pos)
#define ADC_FFSTAT_FULL_Pos			2
#define ADC_FFSTAT_FULL_Msk			(0x01 << ADC_FFSTAT_FULL_Pos)
#define ADC_FFSTAT_EMPTY_Pos		3
#define ADC_FFSTAT_EMPTY_Msk		(0x01 << ADC_FFSTAT_EMPTY_Pos)

#define ADC_FFDATA_VALUE_Pos		0		//������ٴ�ת�������ݻᱻ����
#define ADC_FFDATA_VALUE_Msk		(0xFFF << ADC_FFDATA_VALUE_Pos)
#define ADC_FFDATA_CHNUM_Pos		12
#define ADC_FFDATA_CHNUM_Msk		(0x07 << ADC_FFDATA_CHNUM_Pos)

#define ADC_CTRL1_CLKSRC_Pos		0
#define ADC_CTRL1_CLKSRC_Msk		(0x01 << ADC_CTRL1_CLKSRC_Pos)

#define ADC_CTRL2_ADCEVCM_Pos		1		//ADC External VCM��ADC��PGA�����ģ��ƽѡ��
#define ADC_CTRL2_ADCEVCM_Msk		(0x01 << ADC_CTRL2_ADCEVCM_Pos)
#define ADC_CTRL2_PGAEVCM_Pos		2		//PGA External VCM��PGA���빲ģ��ƽѡ��
#define ADC_CTRL2_PGAEVCM_Msk		(0x01 << ADC_CTRL2_PGAEVCM_Pos)
#define ADC_CTRL2_PGAGAIN_Pos		3
#define ADC_CTRL2_PGAGAIN_Msk		(0x07 << ADC_CTRL2_PGAGAIN_Pos)
#define ADC_CTRL2_VCMSEL_Pos		8
#define ADC_CTRL2_VCMSEL_Msk		(0x07 << ADC_CTRL2_VCMSEL_Pos)
#define ADC_CTRL2_CLKDIV2_Pos		24
#define ADC_CTRL2_CLKDIV2_Msk		(0x1F << ADC_CTRL2_CLKDIV2_Pos)
#define ADC_CTRL2_CLKDIV1_Pos		29
#define ADC_CTRL2_CLKDIV1_Msk		(0x03 << ADC_CTRL2_CLKDIV1_Pos)

#define ADC_CALIBSET_OFFSET_Pos		0
#define ADC_CALIBSET_OFFSET_Msk		(0x1FF<< ADC_CALIBSET_OFFSET_Pos)
#define ADC_CALIBSET_K_Pos			16
#define ADC_CALIBSET_K_Msk			(0x1FF<< ADC_CALIBSET_K_Pos)

#define ADC_CALIBEN_OFFSET_Pos		0
#define ADC_CALIBEN_OFFSET_Msk		(0x01 << ADC_CALIBEN_OFFSET_Pos)
#define ADC_CALIBEN_K_Pos			1
#define ADC_CALIBEN_K_Msk			(0x01 << ADC_CALIBEN_K_Pos)




typedef struct {
	__IO uint32_t MODE;                     //0 ��ͨģʽ��A��B��·����������
                                            //1 ����ģʽ��A��B��·�������PERA��HIGHA���ƣ�B·�����A·��������෴����DZA��DZB����A��B·����������Ƴ�ʱ��
                                            //2 ����ģʽ��ͬ��ͨģʽ����һ�����ں��Զ�ֹͣ
                                            //3 �Գ�ģʽ��A��B��·�������������������������ڲ���һ������������ڣ��ֱ�������һ����Ƶ�ʽ���һ��
                                            //4 �Գƻ���ģʽ���Գ�ģʽ�ͻ���ģʽ���ۺ�
	
	__IO uint32_t PERA;                     //[15:0] ����
	
	__IO uint32_t HIGHA;                    //[15:0] �ߵ�ƽ����ʱ��
	
	__IO uint32_t DZA;                      //[5:0] ���������������Ƴ�ʱ��������С��HIGHA
	
	__IO uint32_t PERB;
	
	__IO uint32_t HIGHB;
	
	__IO uint32_t DZB;
	
	__IO uint32_t INIOUT;                   //Init Output level����ʼ�����ƽ
} PWM_TypeDef;


#define PWM_INIOUT_PWMA_Pos		0		
#define PWM_INIOUT_PWMA_Msk		(0x01 << PWM_INIOUT_PWMA_Pos)
#define PWM_INIOUT_PWMB_Pos		1		
#define PWM_INIOUT_PWMB_Msk		(0x01 << PWM_INIOUT_PWMB_Pos)


typedef struct {
	__IO uint32_t CLKDIV;
    
	     uint32_t RESERVED[3];
    
	__IO uint32_t FORCEH;
    
    __IO uint32_t ADTRG0A;
    __IO uint32_t ADTRG0B;
    
    __IO uint32_t ADTRG1A;
    __IO uint32_t ADTRG1B;
    
    __IO uint32_t ADTRG2A;
    __IO uint32_t ADTRG2B;
    
	     uint32_t RESERVED2[9];
    
	__IO uint32_t HALT;						//ɲ������
    
	__IO uint32_t CHEN;
    
	__IO uint32_t IE;
    
	__IO uint32_t IF;
    
	__IO uint32_t IMSK;
    
	__IO uint32_t IRAWST;
} PWMG_TypeDef;


#define PWMG_FORCEH_PWM0_Pos		0		
#define PWMG_FORCEH_PWM0_Msk		(0x01 << PWMG_FORCEH_PWM0_Pos)
#define PWMG_FORCEH_PWM1_Pos		1		
#define PWMG_FORCEH_PWM1_Msk		(0x01 << PWMG_FORCEH_PWM1_Pos)
#define PWMG_FORCEH_PWM2_Pos		2		
#define PWMG_FORCEH_PWM2_Msk		(0x01 << PWMG_FORCEH_PWM2_Pos)

#define PWMG_ADTRG0A_VALUE_Pos		0		
#define PWMG_ADTRG0A_VALUE_Msk		(0xFFFF << PWMG_ADTRG0A_VALUE_Pos)
#define PWMG_ADTRG0A_EVEN_Pos		16		
#define PWMG_ADTRG0A_EVEN_Msk		(0x01 << PWMG_ADTRG0A_EVEN_Pos)
#define PWMG_ADTRG0A_EN_Pos		    17		
#define PWMG_ADTRG0A_EN_Msk		    (0x01 << PWMG_ADTRG0A_EN_Pos)
#define PWMG_ADTRG0B_VALUE_Pos		0		
#define PWMG_ADTRG0B_VALUE_Msk		(0xFFFF << PWMG_ADTRG0B_VALUE_Pos)
#define PWMG_ADTRG0B_EVEN_Pos		16		
#define PWMG_ADTRG0B_EVEN_Msk		(0x01 << PWMG_ADTRG0B_EVEN_Pos)
#define PWMG_ADTRG0B_EN_Pos		    17		
#define PWMG_ADTRG0B_EN_Msk		    (0x01 << PWMG_ADTRG0B_EN_Pos)

#define PWMG_ADTRG1A_VALUE_Pos		0		
#define PWMG_ADTRG1A_VALUE_Msk		(0xFFFF << PWMG_ADTRG1A_VALUE_Pos)
#define PWMG_ADTRG1A_EVEN_Pos		16		
#define PWMG_ADTRG1A_EVEN_Msk		(0x01 << PWMG_ADTRG1A_EVEN_Pos)
#define PWMG_ADTRG1A_EN_Pos		    17		
#define PWMG_ADTRG1A_EN_Msk		    (0x01 << PWMG_ADTRG1A_EN_Pos)
#define PWMG_ADTRG1B_VALUE_Pos		0		
#define PWMG_ADTRG1B_VALUE_Msk		(0xFFFF << PWMG_ADTRG1B_VALUE_Pos)
#define PWMG_ADTRG1B_EVEN_Pos		16		
#define PWMG_ADTRG1B_EVEN_Msk		(0x01 << PWMG_ADTRG1B_EVEN_Pos)
#define PWMG_ADTRG1B_EN_Pos		    17		
#define PWMG_ADTRG1B_EN_Msk		    (0x01 << PWMG_ADTRG1B_EN_Pos)

#define PWMG_ADTRG2A_VALUE_Pos		0		
#define PWMG_ADTRG2A_VALUE_Msk		(0xFFFF << PWMG_ADTRG2A_VALUE_Pos)
#define PWMG_ADTRG2A_EVEN_Pos		16		
#define PWMG_ADTRG2A_EVEN_Msk		(0x01 << PWMG_ADTRG2A_EVEN_Pos)
#define PWMG_ADTRG2A_EN_Pos		    17		
#define PWMG_ADTRG2A_EN_Msk		    (0x01 << PWMG_ADTRG2A_EN_Pos)
#define PWMG_ADTRG2B_VALUE_Pos		0		
#define PWMG_ADTRG2B_VALUE_Msk		(0xFFFF << PWMG_ADTRG2B_VALUE_Pos)
#define PWMG_ADTRG2B_EVEN_Pos		16		
#define PWMG_ADTRG2B_EVEN_Msk		(0x01 << PWMG_ADTRG2B_EVEN_Pos)
#define PWMG_ADTRG2B_EN_Pos		    17		
#define PWMG_ADTRG2B_EN_Msk		    (0x01 << PWMG_ADTRG2B_EN_Pos)

#define PWMG_HALT_EN_Pos		    0		
#define PWMG_HALT_EN_Msk		    (0x01 << PWMG_HALT_EN_Pos)
#define PWMG_HALT_PWM0_Pos		    1		
#define PWMG_HALT_PWM0_Msk		    (0x01 << PWMG_HALT_PWM0_Pos)
#define PWMG_HALT_PWM1_Pos		    2		
#define PWMG_HALT_PWM1_Msk		    (0x01 << PWMG_HALT_PWM1_Pos)
#define PWMG_HALT_PWM2_Pos		    3		
#define PWMG_HALT_PWM2_Msk		    (0x01 << PWMG_HALT_PWM2_Pos)
#define PWMG_HALT_STOPCNT_Pos		7		
#define PWMG_HALT_STOPCNT_Msk		(0x01 << PWMG_HALT_STOPCNT_Pos)
#define PWMG_HALT_VALIDI_Pos		8		
#define PWMG_HALT_VALIDI_Msk		(0x01 << PWMG_HALT_VALIDI_Pos)
#define PWMG_HALT_VALIDO_Pos		9		
#define PWMG_HALT_VALIDO_Msk		(0x01 << PWMG_HALT_VALIDO_Pos)
#define PWMG_HALT_STAT_Pos		    10		
#define PWMG_HALT_STAT_Msk		    (0x01 << PWMG_HALT_STAT_Pos)

#define PWMG_CHEN_PWM0A_Pos		    0		
#define PWMG_CHEN_PWM0A_Msk		    (0x01 << PWMG_CHEN_PWM0A_Pos)
#define PWMG_CHEN_PWM0B_Pos		    1		
#define PWMG_CHEN_PWM0B_Msk		    (0x01 << PWMG_CHEN_PWM0B_Pos)
#define PWMG_CHEN_PWM1A_Pos		    2		
#define PWMG_CHEN_PWM1A_Msk		    (0x01 << PWMG_CHEN_PWM1A_Pos)
#define PWMG_CHEN_PWM1B_Pos		    3		
#define PWMG_CHEN_PWM1B_Msk		    (0x01 << PWMG_CHEN_PWM1B_Pos)
#define PWMG_CHEN_PWM2A_Pos		    4		
#define PWMG_CHEN_PWM2A_Msk		    (0x01 << PWMG_CHEN_PWM2A_Pos)
#define PWMG_CHEN_PWM2B_Pos		    5		
#define PWMG_CHEN_PWM2B_Msk		    (0x01 << PWMG_CHEN_PWM2B_Pos)

#define PWMG_IE_NEWP0A_Pos			0		
#define PWMG_IE_NEWP0A_Msk			(0x01 << PWMG_IE_NEWP0A_Pos)
#define PWMG_IE_NEWP0B_Pos			1		
#define PWMG_IE_NEWP0B_Msk			(0x01 << PWMG_IE_NEWP0B_Pos)
#define PWMG_IE_NEWP1A_Pos			2		
#define PWMG_IE_NEWP1A_Msk			(0x01 << PWMG_IE_NEWP1A_Pos)
#define PWMG_IE_NEWP1B_Pos			3		
#define PWMG_IE_NEWP1B_Msk			(0x01 << PWMG_IE_NEWP1B_Pos)
#define PWMG_IE_NEWP2A_Pos			4		
#define PWMG_IE_NEWP2A_Msk			(0x01 << PWMG_IE_NEWP2A_Pos)
#define PWMG_IE_NEWP2B_Pos			5		
#define PWMG_IE_NEWP2B_Msk			(0x01 << PWMG_IE_NEWP2B_Pos)
#define PWMG_IE_HEND0A_Pos			12		
#define PWMG_IE_HEND0A_Msk			(0x01 << PWMG_IE_HEND0A_Pos)
#define PWMG_IE_HEND0B_Pos			13		
#define PWMG_IE_HEND0B_Msk			(0x01 << PWMG_IE_HEND0B_Pos)
#define PWMG_IE_HEND1A_Pos			14		
#define PWMG_IE_HEND1A_Msk			(0x01 << PWMG_IE_HEND1A_Pos)
#define PWMG_IE_HEND1B_Pos			15		
#define PWMG_IE_HEND1B_Msk			(0x01 << PWMG_IE_HEND1B_Pos)
#define PWMG_IE_HEND2A_Pos			16		
#define PWMG_IE_HEND2A_Msk			(0x01 << PWMG_IE_HEND2A_Pos)
#define PWMG_IE_HEND2B_Pos			17		
#define PWMG_IE_HEND2B_Msk			(0x01 << PWMG_IE_HEND2B_Pos)
#define PWMG_IE_HALT_Pos			24		
#define PWMG_IE_HALT_Msk			(0x01 << PWMG_IE_HALT_Pos)

#define PWMG_IF_NEWP0A_Pos			0		
#define PWMG_IF_NEWP0A_Msk			(0x01 << PWMG_IF_NEWP0A_Pos)
#define PWMG_IF_NEWP0B_Pos			1		
#define PWMG_IF_NEWP0B_Msk			(0x01 << PWMG_IF_NEWP0B_Pos)
#define PWMG_IF_NEWP1A_Pos			2		
#define PWMG_IF_NEWP1A_Msk			(0x01 << PWMG_IF_NEWP1A_Pos)
#define PWMG_IF_NEWP1B_Pos			3		
#define PWMG_IF_NEWP1B_Msk			(0x01 << PWMG_IF_NEWP1B_Pos)
#define PWMG_IF_NEWP2A_Pos			4		
#define PWMG_IF_NEWP2A_Msk			(0x01 << PWMG_IF_NEWP2A_Pos)
#define PWMG_IF_NEWP2B_Pos			5		
#define PWMG_IF_NEWP2B_Msk			(0x01 << PWMG_IF_NEWP2B_Pos)
#define PWMG_IF_HEND0A_Pos			12		
#define PWMG_IF_HEND0A_Msk			(0x01 << PWMG_IF_HEND0A_Pos)
#define PWMG_IF_HEND0B_Pos			13		
#define PWMG_IF_HEND0B_Msk			(0x01 << PWMG_IF_HEND0B_Pos)
#define PWMG_IF_HEND1A_Pos			14		
#define PWMG_IF_HEND1A_Msk			(0x01 << PWMG_IF_HEND1A_Pos)
#define PWMG_IF_HEND1B_Pos			15		
#define PWMG_IF_HEND1B_Msk			(0x01 << PWMG_IF_HEND1B_Pos)
#define PWMG_IF_HEND2A_Pos			16		
#define PWMG_IF_HEND2A_Msk			(0x01 << PWMG_IF_HEND2A_Pos)
#define PWMG_IF_HEND2B_Pos			17		
#define PWMG_IF_HEND2B_Msk			(0x01 << PWMG_IF_HEND2B_Pos)
#define PWMG_IF_HALT_Pos			24		
#define PWMG_IF_HALT_Msk			(0x01 << PWMG_IF_HALT_Pos)

#define PWMG_IMSK_NEWP0A_Pos		0		
#define PWMG_IMSK_NEWP0A_Msk		(0x01 << PWMG_IMSK_NEWP0A_Pos)
#define PWMG_IMSK_NEWP0B_Pos		1		
#define PWMG_IMSK_NEWP0B_Msk		(0x01 << PWMG_IMSK_NEWP0B_Pos)
#define PWMG_IMSK_NEWP1A_Pos		2		
#define PWMG_IMSK_NEWP1A_Msk		(0x01 << PWMG_IMSK_NEWP1A_Pos)
#define PWMG_IMSK_NEWP1B_Pos		3		
#define PWMG_IMSK_NEWP1B_Msk		(0x01 << PWMG_IMSK_NEWP1B_Pos)
#define PWMG_IMSK_NEWP2A_Pos		4		
#define PWMG_IMSK_NEWP2A_Msk		(0x01 << PWMG_IMSK_NEWP2A_Pos)
#define PWMG_IMSK_NEWP2B_Pos		5		
#define PWMG_IMSK_NEWP2B_Msk		(0x01 << PWMG_IMSK_NEWP2B_Pos)
#define PWMG_IMSK_HEND0A_Pos		12		
#define PWMG_IMSK_HEND0A_Msk		(0x01 << PWMG_IMSK_HEND0A_Pos)
#define PWMG_IMSK_HEND0B_Pos		13		
#define PWMG_IMSK_HEND0B_Msk		(0x01 << PWMG_IMSK_HEND0B_Pos)
#define PWMG_IMSK_HEND1A_Pos		14		
#define PWMG_IMSK_HEND1A_Msk		(0x01 << PWMG_IMSK_HEND1A_Pos)
#define PWMG_IMSK_HEND1B_Pos		15		
#define PWMG_IMSK_HEND1B_Msk		(0x01 << PWMG_IMSK_HEND1B_Pos)
#define PWMG_IMSK_HEND2A_Pos		16		
#define PWMG_IMSK_HEND2A_Msk		(0x01 << PWMG_IMSK_HEND2A_Pos)
#define PWMG_IMSK_HEND2B_Pos		17		
#define PWMG_IMSK_HEND2B_Msk		(0x01 << PWMG_IMSK_HEND2B_Pos)
#define PWMG_IMSK_HALT_Pos		    24		
#define PWMG_IMSK_HALT_Msk		    (0x01 << PWMG_IMSK_HALT_Pos)

#define PWMG_IRAWST_NEWP0A_Pos		0		//д1����
#define PWMG_IRAWST_NEWP0A_Msk		(0x01 << PWMG_IRAWST_NEWP0A_Pos)
#define PWMG_IRAWST_NEWP0B_Pos		1		//д1����
#define PWMG_IRAWST_NEWP0B_Msk		(0x01 << PWMG_IRAWST_NEWP0B_Pos)
#define PWMG_IRAWST_NEWP1A_Pos		2		//д1����
#define PWMG_IRAWST_NEWP1A_Msk		(0x01 << PWMG_IRAWST_NEWP1A_Pos)
#define PWMG_IRAWST_NEWP1B_Pos		3		//д1����
#define PWMG_IRAWST_NEWP1B_Msk		(0x01 << PWMG_IRAWST_NEWP1B_Pos)
#define PWMG_IRAWST_NEWP2A_Pos		4		//д1����
#define PWMG_IRAWST_NEWP2A_Msk		(0x01 << PWMG_IRAWST_NEWP2A_Pos)
#define PWMG_IRAWST_NEWP2B_Pos		5		//д1����
#define PWMG_IRAWST_NEWP2B_Msk		(0x01 << PWMG_IRAWST_NEWP2B_Pos)
#define PWMG_IRAWST_HEND0A_Pos		12		//д1����
#define PWMG_IRAWST_HEND0A_Msk		(0x01 << PWMG_IRAWST_HEND0A_Pos)
#define PWMG_IRAWST_HEND0B_Pos		13		//д1����
#define PWMG_IRAWST_HEND0B_Msk		(0x01 << PWMG_IRAWST_HEND0B_Pos)
#define PWMG_IRAWST_HEND1A_Pos		14		//д1����
#define PWMG_IRAWST_HEND1A_Msk		(0x01 << PWMG_IRAWST_HEND1A_Pos)
#define PWMG_IRAWST_HEND1B_Pos		15		//д1����
#define PWMG_IRAWST_HEND1B_Msk		(0x01 << PWMG_IRAWST_HEND1B_Pos)
#define PWMG_IRAWST_HEND2A_Pos		16		//д1����
#define PWMG_IRAWST_HEND2A_Msk		(0x01 << PWMG_IRAWST_HEND2A_Pos)
#define PWMG_IRAWST_HEND2B_Pos		17		//д1����
#define PWMG_IRAWST_HEND2B_Msk		(0x01 << PWMG_IRAWST_HEND2B_Pos)
#define PWMG_IRAWST_HALT_Pos		24		//д1����
#define PWMG_IRAWST_HALT_Msk		(0x01 << PWMG_IRAWST_HALT_Pos)




typedef struct {
	__IO uint32_t EN;
    
	__IO uint32_t IE;						//ֻ��IE[n]Ϊ1ʱ��IF[n]��DMA�������ʱ���ܱ�Ϊ1
    
	__IO uint32_t IM;						//ֻ��IM[n]Ϊ0ʱ��IF[n]���1ʱ���ܴ���DMA�ж�
    
	__IO uint32_t IF;
    
	struct {
		__IO uint32_t CR;
		__IO uint32_t SRC;					//Դ��ַ
		__IO uint32_t DST;					//Ŀ�ĵ�ַ
	} CH[6];								//[0] SPI����    [1] SPI����    [2] ��ADCͨ��    [5] ��CANͨ��
} DMA_TypeDef;


#define DMA_IE_CAN_Pos		    	0		//��CANͨ���ж�ʹ��		
#define DMA_IE_CAN_Msk		    	(0x01 << DMA_IE_CAN_Pos)
#define DMA_IE_ADC_Pos			    2		//��ADCͨ���ж�ʹ��
#define DMA_IE_ADC_Msk			    (0x01 << DMA_IE_ADC_Pos)
#define DMA_IE_SPIRX_Pos			4		//��SPIͨ���ж�ʹ��
#define DMA_IE_SPIRX_Msk			(0x01 << DMA_IE_SPIRX_Pos)
#define DMA_IE_SPITX_Pos			5		//дSPIͨ���ж�ʹ��
#define DMA_IE_SPITX_Msk			(0x01 << DMA_IE_SPITX_Pos)

#define DMA_IM_CAN_Pos		    	0
#define DMA_IM_CAN_Msk		    	(0x01 << DMA_IM_CAN_Pos)
#define DMA_IM_ADC_Pos			    2
#define DMA_IM_ADC_Msk			    (0x01 << DMA_IM_ADC_Pos)
#define DMA_IM_SPIRX_Pos			4
#define DMA_IM_SPIRX_Msk			(0x01 << DMA_IM_SPIRX_Pos)
#define DMA_IM_SPITX_Pos			5
#define DMA_IM_SPITX_Msk			(0x01 << DMA_IM_SPITX_Pos)

#define DMA_IF_CAN_Pos		    	0		//д1����
#define DMA_IF_CAN_Msk		    	(0x01 << DMA_IF_CAN_Pos)
#define DMA_IF_ADC_Pos			    2		//д1����
#define DMA_IF_ADC_Msk			    (0x01 << DMA_IF_ADC_Pos)
#define DMA_IF_SPIRX_Pos			4
#define DMA_IF_SPIRX_Msk			(0x01 << DMA_IF_SPIRX_Pos)
#define DMA_IF_SPITX_Pos			5
#define DMA_IF_SPITX_Msk			(0x01 << DMA_IF_SPITX_Pos)

#define DMA_CR_LEN_Pos				0		//DMA�����ֽڸ���
#define DMA_CR_LEN_Msk				(0xFFF << DMA_CR_LEN_Pos)
#define DMA_CR_REN_Pos				16		//��SPIͨ������ADCͨ������CANͨ����ͨ��ʹ��λ
#define DMA_CR_REN_Msk				(0x01 << DMA_CR_REN_Pos)
#define DMA_CR_WEN_Pos				16		//дSPIͨ����ͨ��ʹ��λ��ע�⣺дSPIͨ���ķ���ʹ��Ҳ��16λ������ͨ���ķ���ʹ���ڵ�17λ��������ͨ�����Ƕ�ͨ������ʹ�÷���ʹ��λ	
#define DMA_CR_WEN_Msk				(0x01 << DMA_CR_WEN_Pos)




typedef struct {
	__IO uint32_t CR;						//Control Register
	
	__O  uint32_t CMD;						//Command Register
	
	__I  uint32_t SR;						//Status Register
	
	__I  uint32_t IF;						//Interrupt Flag
	
	__IO uint32_t IE;						//Interrupt Enable
	
	     uint32_t RESERVED;
	
	__IO uint32_t BT0;						//Bit Time Register 0
	
	__IO uint32_t BT1;						//Bit Time Register 1
	
	     uint32_t RESERVED2[3];
	
	__I  uint32_t ALC;						//Arbitration Lost Capture, �ٲö�ʧ��׽
	
	__I  uint32_t ECC;						//Error code capture, ������벶׽
	
	__IO uint32_t EWLIM;					//Error Warning Limit, ���󱨾�����
	
	__IO uint32_t RXERR;					//RX�������
	
	__IO uint32_t TXERR;					//TX�������
	
	union {
		struct {							//�ڸ�λʱ�ɶ�д����������ģʽ�²��ɷ���
			__IO uint32_t ACR[4];			//Acceptance Check Register, ���ռĴ���
			
			__IO uint32_t AMR[4];			//Acceptance Mask Register, �������μĴ�����0 must match    1 don't care
			
				 uint32_t RESERVED[5];
		} FILTER;
		
		union {								//����������ģʽ�¿ɶ�д����λʱ���ɷ���
			struct {
				__O  uint32_t INFO;
				
				__O  uint32_t DATA[12];
			} TXFRAME;
			
			struct {
				__I  uint32_t INFO;
				
				__I  uint32_t DATA[12];
			} RXFRAME;
		};
	};
	
	__I  uint32_t RMCNT;					//Receive Message Count
	
		 uint32_t RESERVED3[66];
	
	struct {								//TXFRAME�Ķ��ӿ�
		__I  uint32_t INFO;
		
		__I  uint32_t DATA[12];
	} TXFRAME_R;
} CAN_TypeDef;


#define CAN_CR_RST_Pos				0
#define CAN_CR_RST_Msk				(0x01 << CAN_CR_RST_Pos)
#define CAN_CR_LOM_Pos				1		//Listen Only Mode
#define CAN_CR_LOM_Msk				(0x01 << CAN_CR_LOM_Pos)
#define CAN_CR_STM_Pos				2		//Self Test Mode, ��ģʽ�¼�ʹû��Ӧ��CAN������Ҳ���Գɹ�����
#define CAN_CR_STM_Msk				(0x01 << CAN_CR_STM_Pos)
#define CAN_CR_AFM_Pos				3		//Acceptance Filter Mode, 1 ���������˲�����32λ��   0 ���������˲�����16λ��
#define CAN_CR_AFM_Msk				(0x01 << CAN_CR_AFM_Pos)
#define CAN_CR_SLEEP_Pos			4		//д1����˯��ģʽ�������߻���ж�ʱ���Ѳ��Զ������λ
#define CAN_CR_SLEEP_Msk			(0x01 << CAN_CR_SLEEP_Pos)
#define CAN_CR_DMAEN_Pos			5
#define CAN_CR_DMAEN_Msk			(0x01 << CAN_CR_DMAEN_Pos)

#define CAN_CMD_TXREQ_Pos			0		//Transmission Request
#define CAN_CMD_TXREQ_Msk			(0x01 << CAN_CMD_TXREQ_Pos)
#define CAN_CMD_ABTTX_Pos			1		//Abort Transmission
#define CAN_CMD_ABTTX_Msk			(0x01 << CAN_CMD_ABTTX_Pos)
#define CAN_CMD_RRB_Pos				2		//Release Receive Buffer
#define CAN_CMD_RRB_Msk				(0x01 << CAN_CMD_RRB_Pos)
#define CAN_CMD_CLROV_Pos			3		//Clear Data Overrun
#define CAN_CMD_CLROV_Msk			(0x01 << CAN_CMD_CLROV_Pos)
#define CAN_CMD_SRR_Pos				4		//Self Reception Request
#define CAN_CMD_SRR_Msk				(0x01 << CAN_CMD_SRR_Pos)

#define CAN_SR_RXDA_Pos				0		//Receive Data Available������FIFO����������Ϣ���Զ�ȡ
#define CAN_SR_RXDA_Msk				(0x01 << CAN_SR_RXDA_Pos)
#define CAN_SR_RXOV_Pos				1		//Receive FIFO Overrun���½��յ���Ϣ���ڽ���FIFO����������
#define CAN_SR_RXOV_Msk				(0x01 << CAN_SR_RXOV_Pos)
#define CAN_SR_TXBR_Pos				2		//Transmit Buffer Release��0 ���ڴ���ǰ��ķ��ͣ����ڲ���д�µ���Ϣ    1 ����д���µ���Ϣ����
#define CAN_SR_TXBR_Msk				(0x01 << CAN_SR_TXBR_Pos)
#define CAN_SR_TXOK_Pos				3		//Transmit OK��successfully completed
#define CAN_SR_TXOK_Msk				(0x01 << CAN_SR_TXOK_Pos)
#define CAN_SR_RXBUSY_Pos			4		//Receive Busy�����ڽ���
#define CAN_SR_RXBUSY_Msk			(0x01 << CAN_SR_RXBUSY_Pos)
#define CAN_SR_TXBUSY_Pos			5		//Transmit Busy�����ڷ���
#define CAN_SR_TXBUSY_Msk			(0x01 << CAN_SR_TXBUSY_Pos)
#define CAN_SR_ERRWARN_Pos			6		//1 ����һ������������ﵽ Warning Limit
#define CAN_SR_ERRWARN_Msk			(0x01 << CAN_SR_ERRWARN_Pos)
#define CAN_SR_BUSOFF_Pos			7		//1 CAN �������������߹ر�״̬��û�в��뵽���߻
#define CAN_SR_BUSOFF_Msk			(0x01 << CAN_SR_BUSOFF_Pos)

#define CAN_IF_RXDA_Pos				0		//IF.RXDA = SR.RXDA & IE.RXDA
#define CAN_IF_RXDA_Msk				(0x01 << CAN_IF_RXDA_Pos)
#define CAN_IF_TXBR_Pos				1		//��IE.TXBR=1ʱ��SR.TXBR��0���1����λ��λ
#define CAN_IF_TXBR_Msk				(0x01 << CAN_IF_TXBR_Pos)
#define CAN_IF_ERRWARN_Pos			2		//��IE.ERRWARN=1ʱ��SR.ERRWARN��SR.BUSOFF 0-to-1 �� 1-to-0����λ��λ
#define CAN_IF_ERRWARN_Msk			(0x01 << CAN_IF_ERRWARN_Pos)
#define CAN_IF_RXOV_Pos				3		//IF.RXOV = SR.RXOV & IE.RXOV
#define CAN_IF_RXOV_Msk				(0x01 << CAN_IF_RXOV_Pos)
#define CAN_IF_WKUP_Pos				4		//��IE.WKUP=1ʱ����˯��ģʽ�µ�CAN��������⵽���߻ʱӲ����λ
#define CAN_IF_WKUP_Msk				(0x01 << CAN_IF_WKUP_Pos)
#define CAN_IF_ERRPASS_Pos			5		//
#define CAN_IF_ERRPASS_Msk			(0x01 << CAN_IF_ERRPASS_Pos)
#define CAN_IF_ARBLOST_Pos			6		//Arbitration Lost����IE.ARBLOST=1ʱ��CAN��������ʧ�ٲñ�ɽ��շ�ʱӲ����λ
#define CAN_IF_ARBLOST_Msk			(0x01 << CAN_IF_ARBLOST_Pos)
#define CAN_IF_BUSERR_Pos			7		//��IE.BUSERR=1ʱ��CAN��������⵽���ߴ���ʱӲ����λ
#define CAN_IF_BUSERR_Msk			(0x01 << CAN_IF_BUSERR_Pos)

#define CAN_IE_RXDA_Pos				0
#define CAN_IE_RXDA_Msk				(0x01 << CAN_IE_RXDA_Pos)
#define CAN_IE_TXBR_Pos				1
#define CAN_IE_TXBR_Msk				(0x01 << CAN_IE_TXBR_Pos)
#define CAN_IE_ERRWARN_Pos			2
#define CAN_IE_ERRWARN_Msk			(0x01 << CAN_IE_ERRWARN_Pos)
#define CAN_IE_RXOV_Pos				3
#define CAN_IE_RXOV_Msk				(0x01 << CAN_IE_RXOV_Pos)
#define CAN_IE_WKUP_Pos				4
#define CAN_IE_WKUP_Msk				(0x01 << CAN_IE_WKUP_Pos)
#define CAN_IE_ERRPASS_Pos			5
#define CAN_IE_ERRPASS_Msk			(0x01 << CAN_IE_ERRPASS_Pos)
#define CAN_IE_ARBLOST_Pos			6
#define CAN_IE_ARBLOST_Msk			(0x01 << CAN_IE_ARBLOST_Pos)
#define CAN_IE_BUSERR_Pos			7
#define CAN_IE_BUSERR_Msk			(0x01 << CAN_IE_BUSERR_Pos)

#define CAN_BT0_BRP_Pos				0		//Baud Rate Prescaler��CANʱ�䵥λ=2*Tsysclk*(BRP+1)
#define CAN_BT0_BRP_Msk				(0x3F << CAN_BT0_BRP_Pos)
#define CAN_BT0_SJW_Pos				6		//Synchronization Jump Width
#define CAN_BT0_SJW_Msk				(0x03 << CAN_BT0_SJW_Pos)

#define CAN_BT1_TSEG1_Pos			0		//t_tseg1 = CANʱ�䵥λ * (TSEG1+1)
#define CAN_BT1_TSEG1_Msk			(0x0F << CAN_BT1_TSEG1_Pos)
#define CAN_BT1_TSEG2_Pos			4		//t_tseg2 = CANʱ�䵥λ * (TSEG2+1)
#define CAN_BT1_TSEG2_Msk			(0x07 << CAN_BT1_TSEG2_Pos)
#define CAN_BT1_SAM_Pos				7		//��������  0: sampled once  1: sampled three times
#define CAN_BT1_SAM_Msk				(0x01 << CAN_BT1_SAM_Pos)

#define CAN_ECC_SEGCODE_Pos			0		//Segment Code
#define CAN_ECC_SEGCODE_Msk			(0x1F << CAN_ECC_SEGCODE_Pos)
#define CAN_ECC_DIR_Pos				5		//0 error occurred during transmission   1 during reception
#define CAN_ECC_DIR_Msk				(0x01 << CAN_ECC_DIR_Pos)
#define CAN_ECC_ERRCODE_Pos			6		//Error Code��0 Bit error   1 Form error   2 Stuff error   3 other error
#define CAN_ECC_ERRCODE_Msk			(0x03 << CAN_ECC_ERRCODE_Pos)

#define CAN_INFO_DLC_Pos			0		//Data Length Control
#define CAN_INFO_DLC_Msk			(0x0F << CAN_INFO_DLC_Pos)
#define CAN_INFO_RTR_Pos			6		//Remote Frame��1 Զ��֡    0 ����֡
#define CAN_INFO_RTR_Msk			(0x01 << CAN_INFO_RTR_Pos)
#define CAN_INFO_FF_Pos				7		//Frame Format��0 ��׼֡��ʽ    1 ��չ֡��ʽ
#define CAN_INFO_FF_Msk				(0x01 << CAN_INFO_FF_Pos)




typedef struct {
	__IO uint32_t LOAD;						//ι��ʹ������װ��LOADֵ
	
	__I  uint32_t VALUE;
	
	__IO uint32_t CR;
	
	__IO uint32_t IF;						//������0ʱӲ����λ�����д0�����־
	
	__IO uint32_t FEED;						//д0x55ι��
} WDT_TypeDef;


#define WDT_CR_EN_Pos				0
#define WDT_CR_EN_Msk				(0x01 << WDT_CR_EN_Pos)
#define WDT_CR_RSTEN_Pos			1
#define WDT_CR_RSTEN_Msk			(0x01 << WDT_CR_RSTEN_Pos)




typedef struct {
	__IO uint32_t CR;
	
	__IO uint32_t SR;
	
	     uint32_t RESERVED[2];
	
	__IO uint32_t DIVIDEND;					//������
	
	__IO uint32_t DIVISOR;					//����
	
	__IO uint32_t QUO;						//��
	
	__IO uint32_t REMAIN;					//����
	
	__IO uint32_t RADICAND;					//��������
	
	__IO uint32_t ROOT;						//ƽ��������16λΪС�����֣���16λΪ��������
} DIV_TypeDef;


#define DIV_CR_DIVGO_Pos			0		//д1�����������㣬������ɺ��Զ�����
#define DIV_CR_DIVGO_Msk			(0x01 << DIV_CR_DIVGO_Pos)
#define DIV_CR_ROOTGO_Pos			8		//д1������ƽ�������㣬������ɺ��Զ�����
#define DIV_CR_ROOTGO_Msk			(0x01 << DIV_CR_ROOTGO_Pos)
#define DIV_CR_ROOTMOD_Pos			9		//��ƽ����ģʽ��0 ���Ϊ����    1 �������������������С������
#define DIV_CR_ROOTMOD_Msk			(0x01 << DIV_CR_ROOTMOD_Pos)

#define DIV_SR_DIVEND_Pos			0		//����������ɱ�־��д1����
#define DIV_SR_DIVEND_Msk			(0x01 << DIV_SR_DIVEND_Pos)
#define DIV_SR_DIVBUSY_Pos			1		//1 �������������
#define DIV_SR_DIVBUSY_Msk			(0x01 << DIV_SR_DIVBUSY_Pos)
#define DIV_SR_ROOTENDI_Pos			8		//��������������ɱ�־��д1����
#define DIV_SR_ROOTENDI_Msk			(0x01 << DIV_SR_ROOTENDI_Pos)
#define DIV_SR_ROOTENDF_Pos			9		//����С��������ɱ�־��д1����
#define DIV_SR_ROOTENDF_Msk			(0x01 << DIV_SR_ROOTENDF_Pos)
#define DIV_SR_ROOTBUSY_Pos			10		//1 �������������
#define DIV_SR_ROOTBUSY_Msk			(0x01 << DIV_SR_ROOTBUSY_Pos)




typedef struct {
    __IO uint32_t MINSEC;                   //�������
    
    __IO uint32_t DATHUR;                   //��ʱ����
    
    __IO uint32_t MONDAY;                   //���ܼ���
    
    __IO uint32_t YEAR;                     //[11:0] �����
    
    __IO uint32_t MINSECAL;                 //������������
    
    __IO uint32_t DAYHURAL;                 //��ʱ��������
    
    __IO uint32_t LOAD;
    
    __IO uint32_t IE;
    
    __IO uint32_t IF;                       //д1����
    
    __IO uint32_t EN;                       //[0] 1 RTCʹ��
    
    __IO uint32_t CFGABLE;                  //[0] 1 RTC������
    
    __IO uint32_t TRIM;                     //ʱ�ӵ���
    
    __IO uint32_t TRIMM;                    //ʱ��΢����
	
	     uint32_t RESERVED[11];
	
	__IO uint32_t CALIBREFCNT;				//[23:0] У׼��׼ʱ��Ƶ�ʵ�һ�룬У׼��׼ʱ��ȡֵ��Χ2.2~4.1MHz
	
	__IO uint32_t CALIBEN;					//[0] ����У׼��У׼��ɺ��Զ�����
	
	__I  uint32_t CALIBST;
} RTC_TypeDef;


#define RTC_MINSEC_SEC_Pos			0       //�������ȡֵ0--59
#define RTC_MINSEC_SEC_Msk		    (0x3F << RTC_MINSEC_SEC_Pos)
#define RTC_MINSEC_MIN_Pos			6       //���Ӽ�����ȡֵ0--59
#define RTC_MINSEC_MIN_Msk		    (0x3F << RTC_MINSEC_MIN_Pos)

#define RTC_DATHUR_HOUR_Pos			0       //Сʱ������ȡֵ0--23
#define RTC_DATHUR_HOUR_Msk		    (0x1F << RTC_DATHUR_HOUR_Pos)
#define RTC_DATHUR_DATE_Pos			5       //date of month��ȡֵ1--31
#define RTC_DATHUR_DATE_Msk		    (0x1F << RTC_DATHUR_DATE_Pos)

#define RTC_MONDAY_DAY_Pos			0       //day of week��ȡֵ0--6
#define RTC_MONDAY_DAY_Msk		    (0x07 << RTC_MONDAY_DAY_Pos)
#define RTC_MONDAY_MON_Pos			3       //�·ݼ�����ȡֵ1--12
#define RTC_MONDAY_MON_Msk		    (0x0F << RTC_MONDAY_MON_Pos)

#define RTC_MINSECAL_SEC_Pos		0       //����������
#define RTC_MINSECAL_SEC_Msk		(0x3F << RTC_MINSECAL_SEC_Pos)
#define RTC_MINSECAL_MIN_Pos	    6       //���ӷ�������
#define RTC_MINSECAL_MIN_Msk		(0x3F << RTC_MINSECAL_MIN_Pos)

#define RTC_DAYHURAL_HOUR_Pos		0       //����Сʱ����
#define RTC_DAYHURAL_HOUR_Msk		(0x1F << RTC_DAYHURAL_HOUR_Pos)
#define RTC_DAYHURAL_SUN_Pos		5       //����������Ч
#define RTC_DAYHURAL_SUN_Msk		(0x01 << RTC_DAYHURAL_SUN_Pos)
#define RTC_DAYHURAL_MON_Pos		6       //��һ������Ч
#define RTC_DAYHURAL_MON_Msk		(0x01 << RTC_DAYHURAL_MON_Pos)
#define RTC_DAYHURAL_TUE_Pos		7       //�ܶ�������Ч
#define RTC_DAYHURAL_TUE_Msk		(0x01 << RTC_DAYHURAL_TUE_Pos)
#define RTC_DAYHURAL_WED_Pos		8       //����������Ч
#define RTC_DAYHURAL_WED_Msk		(0x01 << RTC_DAYHURAL_WED_Pos)
#define RTC_DAYHURAL_THU_Pos		9       //����������Ч
#define RTC_DAYHURAL_THU_Msk		(0x01 << RTC_DAYHURAL_THU_Pos)
#define RTC_DAYHURAL_FRI_Pos		10      //����������Ч
#define RTC_DAYHURAL_FRI_Msk		(0x01 << RTC_DAYHURAL_FRI_Pos)
#define RTC_DAYHURAL_SAT_Pos		11      //����������Ч
#define RTC_DAYHURAL_SAT_Msk		(0x01 << RTC_DAYHURAL_SAT_Pos)

#define RTC_LOAD_TIME_Pos			0		//��ʱ�䡢��ʱ��У׼��ؼĴ����е�ֵ���ص�RTCʱ����
#define RTC_LOAD_TIME_Msk			(0x01 << RTC_LOAD_TIME_Pos)
#define RTC_LOAD_ALARM_Pos			1		//��������ؼĴ����е�ֵ���ص�RTCʱ����
#define RTC_LOAD_ALARM_Msk			(0x01 << RTC_LOAD_ALARM_Pos)

#define RTC_IE_SEC_Pos		        0       //���ж�ʹ��
#define RTC_IE_SEC_Msk		        (0x01 << RTC_IE_SEC_Pos)
#define RTC_IE_MIN_Pos		        1
#define RTC_IE_MIN_Msk		        (0x01 << RTC_IE_MIN_Pos)
#define RTC_IE_HOUR_Pos		        2
#define RTC_IE_HOUR_Msk		        (0x01 << RTC_IE_HOUR_Pos)
#define RTC_IE_DATE_Pos		        3
#define RTC_IE_DATE_Msk		        (0x01 << RTC_IE_DATE_Pos)
#define RTC_IE_ALARM_Pos		    4
#define RTC_IE_ALARM_Msk		    (0x01 << RTC_IE_ALARM_Pos)

#define RTC_IF_SEC_Pos		        0       //д1����
#define RTC_IF_SEC_Msk		        (0x01 << RTC_IF_SEC_Pos)
#define RTC_IF_MIN_Pos		        1
#define RTC_IF_MIN_Msk		        (0x01 << RTC_IF_MIN_Pos)
#define RTC_IF_HOUR_Pos		        2
#define RTC_IF_HOUR_Msk		        (0x01 << RTC_IF_HOUR_Pos)
#define RTC_IF_DATE_Pos		        3
#define RTC_IF_DATE_Msk		        (0x01 << RTC_IF_DATE_Pos)
#define RTC_IF_ALARM_Pos		    4
#define RTC_IF_ALARM_Msk		    (0x01 << RTC_IF_ALARM_Pos)

#define RTC_TRIM_ADJ_Pos		    0       //���ڵ���BASECNT�ļ������ڣ�Ĭ��Ϊ32768�����DECΪ1����������ڵ���Ϊ32768-ADJ���������Ϊ32768+ADJ
#define RTC_TRIM_ADJ_Msk		    (0xFF << RTC_TRIM_ADJ_Pos)
#define RTC_TRIM_DEC_Pos		    8
#define RTC_TRIM_DEC_Msk		    (0x01 << RTC_TRIM_DEC_Pos)

#define RTC_TRIMM_CYCLE_Pos		    0       //���ڼ�������΢�������INCΪ1�����n���������ڵ���Ϊ(32768��ADJ)+1,�������Ϊ(32768��ADJ)-1
                                            //cycles=0ʱ��������΢������cycles=1����nΪ2��cycles=7����nΪ8���Դ�����
#define RTC_TRIMM_CYCLE_Msk		    (0x07 << RTC_TRIMM_CYCLE_Pos)
#define RTC_TRIMM_INC_Pos		    3
#define RTC_TRIMM_INC_Msk		    (0x01 << RTC_TRIMM_INC_Pos)




/******************************************************************************/
/*						 Peripheral memory map							  */
/******************************************************************************/
#define RAM_BASE		   	0x20000000
#define AHB_BASE			0x40000000
#define APB_BASE		 	0x50000000

/* AHB Peripheral memory map */
#define SYS_BASE			(AHB_BASE + 0x0000000)
#define DMA_BASE			(AHB_BASE + 0x1000000)
#define DIV_BASE			(AHB_BASE + 0x6000000)

/* APB Peripheral memory map */
#define PORTA_BASE			(APB_BASE +	0x0000)
#define PORTB_BASE			(APB_BASE +	0x0010)
#define PORTC_BASE			(APB_BASE +	0x0020)
#define PORTD_BASE			(APB_BASE +	0x0030)

#define GPIOA_BASE			(APB_BASE + 0x01000)
#define GPIOB_BASE			(APB_BASE + 0x02000)
#define GPIOC_BASE			(APB_BASE + 0x03000)
#define GPIOD_BASE			(APB_BASE + 0x04000)

#define TIMR0_BASE			(APB_BASE +	0x07000)
#define TIMR1_BASE			(APB_BASE +	0x0700C)
#define TIMR2_BASE			(APB_BASE +	0x07018)
#define TIMR3_BASE			(APB_BASE +	0x07024)
#define TIMRG_BASE			(APB_BASE +	0x07060)

#define WDT_BASE			(APB_BASE + 0x09000)

#define PWM0_BASE			(APB_BASE + 0x0A000)
#define PWM1_BASE			(APB_BASE + 0x0A020)
#define PWM2_BASE			(APB_BASE + 0x0A040)
#define PWMG_BASE			(APB_BASE + 0x0A170)

#define RTC_BASE			(APB_BASE + 0x0B000)

#define ADC_BASE			(APB_BASE + 0x0D000)

#define UART0_BASE			(APB_BASE + 0x10000)
#define UART1_BASE			(APB_BASE + 0x11000)
#define UART2_BASE			(APB_BASE + 0x12000)
#define UART3_BASE			(APB_BASE + 0x13000)

#define I2C0_BASE			(APB_BASE + 0x18000)
#define I2C1_BASE			(APB_BASE + 0x19000)

#define SPI0_BASE			(APB_BASE + 0x1C000)
#define SPI1_BASE			(APB_BASE + 0x1E000)

#define CAN_BASE			(APB_BASE + 0x20000)


/******************************************************************************/
/*						 Peripheral declaration							 */
/******************************************************************************/
#define SYS					((SYS_TypeDef  *) SYS_BASE)

#define PORTA				((PORT_TypeDef *) PORTA_BASE)
#define PORTB				((PORT_TypeDef *) PORTB_BASE)
#define PORTC				((PORT_TypeDef *) PORTC_BASE)
#define PORTD				((PORT_TypeDef *) PORTD_BASE)

#define GPIOA				((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB				((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC				((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD				((GPIO_TypeDef *) GPIOD_BASE)

#define TIMR0				((TIMR_TypeDef *) TIMR0_BASE)
#define TIMR1				((TIMR_TypeDef *) TIMR1_BASE)
#define TIMR2				((TIMR_TypeDef *) TIMR2_BASE)
#define TIMR3				((TIMR_TypeDef *) TIMR3_BASE)
#define TIMRG				((TIMRG_TypeDef*) TIMRG_BASE)

#define UART0				((UART_TypeDef *) UART0_BASE)
#define UART1				((UART_TypeDef *) UART1_BASE)
#define UART2				((UART_TypeDef *) UART2_BASE)
#define UART3   			((UART_TypeDef *) UART3_BASE)

#define SPI0				((SPI_TypeDef  *) SPI0_BASE)
#define SPI1				((SPI_TypeDef  *) SPI1_BASE)

#define I2C0				((I2C_TypeDef  *) I2C0_BASE)
#define I2C1				((I2C_TypeDef  *) I2C1_BASE)

#define ADC 				((ADC_TypeDef  *) ADC_BASE)

#define PWM0				((PWM_TypeDef  *) PWM0_BASE)
#define PWM1				((PWM_TypeDef  *) PWM1_BASE)
#define PWM2				((PWM_TypeDef  *) PWM2_BASE)
#define PWMG				((PWMG_TypeDef *) PWMG_BASE)

#define DMA 				((DMA_TypeDef  *) DMA_BASE)

#define CAN 				((CAN_TypeDef  *) CAN_BASE)

#define WDT					((WDT_TypeDef  *) WDT_BASE)

#define	DIV					((DIV_TypeDef  *) DIV_BASE)

#define	RTC					((RTC_TypeDef  *) RTC_BASE)


#include "SWM220_port.h"
#include "SWM220_gpio.h"
#include "SWM220_exti.h"
#include "SWM220_timr.h"
#include "SWM220_uart.h"
#include "SWM220_spi.h"
#include "SWM220_i2c.h"
#include "SWM220_pwm.h"
#include "SWM220_adc.h"
#include "SWM220_dma.h"
#include "SWM220_can.h"
#include "SWM220_flash.h"
#include "SWM220_wdt.h"
#include "SWM220_rtc.h"
#include "SWM220_div.h"
#include "SWM220_sleep.h"

#endif //__SWM220_H__
