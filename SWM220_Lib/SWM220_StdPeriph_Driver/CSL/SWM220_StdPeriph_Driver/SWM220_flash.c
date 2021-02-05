/****************************************************************************************************************************************** 
* �ļ�����:	SWM220_flash.c
* ����˵��:	ʹ��оƬ��IAP���ܽ�Ƭ��Flashģ���EEPROM���������ݣ�����󲻶�ʧ
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
* �汾����: V1.0.0		2016��1��30��
* ������¼: 
*******************************************************************************************************************************************
* @attention
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS WITH CODING INFORMATION 
* REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME. AS A RESULT, SYNWIT SHALL NOT BE HELD LIABLE 
* FOR ANY DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT 
* OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION CONTAINED HEREIN IN CONN-
* -ECTION WITH THEIR PRODUCTS.
*
* COPYRIGHT 2012 Synwit Technology  
*******************************************************************************************************************************************/
#include "SWM220.h"
#include "SWM220_flash.h"

typedef uint32_t (*IAPFunc1)(uint32_t addr);
typedef uint32_t (*IAPFunc2)(uint32_t faddr, uint32_t raddr, uint32_t cnt);

IAPFunc1 FLASH_PageErase = (IAPFunc1)0x1000501;

IAPFunc2 FLASH_PageWrite = (IAPFunc2)0x1000601;


/****************************************************************************************************************************************** 
* ��������:	FLASH_Erase()
* ����˵��:	FLASHҳ������ÿ��ҳ1K�ֽ�
* ��    ��: uint32_t addr		Ҫ����ҳ�ĵ�ַ������1K���룬��addr%1024 == 0
* ��    ��: uint32_t			0 �����ɹ�    1 ����ʧ��
* ע������: ��
******************************************************************************************************************************************/
uint32_t FLASH_Erase(uint32_t addr)
{
	uint32_t status;
	
	__disable_irq();
	status = FLASH_PageErase(addr/1024);
	__enable_irq();
	
	return status;
}

/****************************************************************************************************************************************** 
* ��������:	FLASH_Write()
* ����˵��:	FLASH����д��
* ��    ��: uint32_t addr		����Ҫд�뵽Flash�еĵ�ַ���ֶ���
*			uint32_t buff[]		Ҫд��Flash�е�����
*			uint32_t cnt		Ҫд�����ݵĸ���������Ϊ��λ�����ȡֵ256
* ��    ��: uint32_t			0 д��ɹ�    1 д��ʧ��
* ע������: Ҫд������ݱ���ȫ����ͬһҳ�ڣ���addr/1024 == (addr+(cnt-1)*4)/1024
******************************************************************************************************************************************/
uint32_t FLASH_Write(uint32_t addr, uint32_t buff[], uint32_t cnt)
{	
	uint32_t status;
	
	__disable_irq();
	status = FLASH_PageWrite(addr, (uint32_t)buff, cnt);
	__enable_irq();
	
	return status;
}

#if   defined ( __CC_ARM )

/****************************************************************************************************************************************** 
* ��������:	FlashSpeedX2()
* ����˵��:	Flash��ȡ�ٶ����һ��
*			Flash����ٶ�25MHz��CPU����ٶ�48MHz��Ϊ��ʹ��Ƭ���������ٶ��¶��ܶ�ȡFlash��Ĭ��CPUÿ����ʱ�����ڶ�Flash��ȡһ�Σ�
*			ִ�д˺�����CPUһ�����ڶ�ȡFlashһ�Σ�����ִ���ٶ�����60%
* ��    ��: ��
* ��    ��: ��
* ע������: ֻ���ڵ�Ƭ���ٶ�С��25MHz�������ִ�д˺���
******************************************************************************************************************************************/
uint16_t Code_FlashSpeedX2[] = {
	0x6901,     // LDR      r1,[r0,#0x10]
	0x2201,     // MOVS     r2,#0x01
	0x0412,     // LSLS     r2,r2,#16
	0x4311,     // ORRS     r1,r1,r2
	0x6101,     // STR      r1,[r0,#0x10]
	0xBF00,		// NOP
	0xBF00,		// NOP
	0x4770,		// BX LR
};
__asm void FlashSpeedX2(void)
{
	IMPORT Code_FlashSpeedX2
	PUSH {LR}
	LDR R0,=0x50028000		// Flash Controller
	LDR R1,=Code_FlashSpeedX2
    ADDS R1, R1, #1
	NOP
	BLX R1
	POP {R0}
	BX R0
}

#elif defined ( __ICCARM__ )

#endif

__attribute__((section("PlaceInRAM")))
void AppToIsp(void)
{
	uint8_t i = 0;
	__disable_irq();
	(*(volatile unsigned int *)0x40000500) = 0x12345678;
	(*(volatile unsigned int *)0x50028018) = 0x04;  //��flash�л���Info��
	__DMB();  //��֤����ָ�������ָ���ܰ�˳��ִ��
	for(i=0; i<5; i++);	
	SYS->CLKEN = 0x0;  //�ر���������ʱ��	
//	__enable_irq();  //����ִ�������λ���Բ�ʹ�����ж�	
	SCB->AIRCR = ((0x5FA<<SCB_AIRCR_VECTKEY_Pos)|SCB_AIRCR_SYSRESETREQ_Msk);  //ϵͳ�����λ
	__DSB();  /* Ensure completion of memory access */              
	while(1);		
}
