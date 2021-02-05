/****************************************************************************************************************************************** 
* �ļ�����: SWM220_dma.c
* ����˵��:	SWM220��Ƭ����DMA����������
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������: �ڲ�ʹ���жϵ�����£�Ҳ���Ե���DMA_CH_INTStat()��ѯ���ݰ����Ƿ���ɣ�������DMA_CH_INTClr()�����ɱ�־
* �汾����:	V1.0.0		2016��1��30��
* ������¼:  
*
*
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
#include "SWM220_dma.h"


/****************************************************************************************************************************************** 
* ��������: DMA_CH_Config()
* ����˵��:	DMAͨ������
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
*			uint32_t ram_addr		����Ҫ�����˵�RAM�еĵ�ַ�������ֶ���
*			uint32_t num_word		Ҫ���˵�����������ע�⣬��λ���֣������ֽڣ����ֵ1024
*			uint32_t int_en			�ж�ʹ�ܣ�1 ���ݰ�����ɺ�����ж�    0 ���ݰ�����ɺ󲻲����ж�
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_Config(uint32_t chn, uint32_t ram_addr, uint32_t num_word, uint32_t int_en)
{
	DMA->EN = 1;			//ÿ��ͨ�������Լ������Ŀ��ؿ��ƣ������ܿ��ؿ�����һֱ������
	
	DMA_CH_Close(chn);		//����ǰ�ȹرո�ͨ��
	
	switch(chn)
	{
	case DMA_CHW_SPI:
		SPI0->CTRL |= (1 << SPI_CTRL_DMAEN_Pos);
		
		DMA->CH[chn].SRC = ram_addr;
		DMA->CH[chn].DST = (uint32_t)&SPI0->DATA;		//0x5001C004
		break;
	
	case DMA_CHR_SPI:
		SPI0->CTRL |= (1 << SPI_CTRL_DMAEN_Pos);
		
		DMA->CH[chn].SRC = (uint32_t)&SPI0->DATA;		//0x5001C004
		DMA->CH[chn].DST = ram_addr;
		break;
	
	case DMA_CHR_ADC:
		ADC->CTRL |= (0x01 << ADC_CTRL_DMAEN_Pos) | (0x01 << ADC_CTRL_RES2FF_Pos);
		
		DMA->CH[chn].SRC = (uint32_t)&ADC->FFDATA;		//0x5000D094
		DMA->CH[chn].DST = ram_addr;
		break;
	
	case DMA_CHR_CAN:
		CAN->CR |= (0x01 << CAN_CR_DMAEN_Pos);
		
		DMA->CH[chn].SRC = (uint32_t)&CAN->RXFRAME;		//0x50020040
		DMA->CH[chn].DST = ram_addr;
		break;
	}
	
	DMA->CH[chn].CR = ((num_word*4-1) << DMA_CR_LEN_Pos);
	
	DMA->IE = 0x35;			//��ʹ��ʹ���жϣ�Ҳ���Բ�ѯ״̬/��־
	DMA_CH_INTClr(chn);		//����жϱ�־
	if(int_en) DMA_CH_INTEn(chn);
	else	   DMA_CH_INTDis(chn);
	
	if(int_en)
	{
		NVIC_EnableIRQ(DMA_IRQn);
	}
	else
	{		
		//����NVIC_DisableIRQ(DMA_IRQn); ��Ϊ����ͨ������ʹ���ж�
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_Open()
* ����˵��:	DMAͨ����
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_Open(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_SPI:
		DMA->CH[chn].CR |= (1 << DMA_CR_WEN_Pos);
		break;
	
	case DMA_CHR_SPI:
	case DMA_CHR_ADC:		
	case DMA_CHR_CAN:
		DMA->CH[chn].CR |= (1 << DMA_CR_REN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_Close()
* ����˵��:	DMAͨ���ر�
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_Close(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_SPI:
		DMA->CH[chn].CR &= ~(1 << DMA_CR_WEN_Pos);
		break;
	
	case DMA_CHR_SPI:
	case DMA_CHR_ADC:		
	case DMA_CHR_CAN:
		DMA->CH[chn].CR &= ~(1 << DMA_CR_REN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_INTEn()
* ����˵��:	DMA�ж�ʹ�ܣ����ݰ�����ɺ󴥷��ж�
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_INTEn(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_SPI:
		DMA->IM &= ~(1 << DMA_IM_SPITX_Pos);
		break;
	
	case DMA_CHR_SPI:
		DMA->IM &= ~(1 << DMA_IM_SPIRX_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IM &= ~(1 << DMA_IM_ADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IM &= ~(1 << DMA_IM_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_INTDis()
* ����˵��:	DMA�жϽ�ֹ�����ݰ�����ɺ󲻴����ж�
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_INTDis(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_SPI:
		DMA->IM |= (1 << DMA_IM_SPITX_Pos);
		break;
	
	case DMA_CHR_SPI:
		DMA->IM |= (1 << DMA_IM_SPIRX_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IM |= (1 << DMA_IM_ADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IM |= (1 << DMA_IM_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_INTClr()
* ����˵��:	DMA�жϱ�־���
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void DMA_CH_INTClr(uint32_t chn)
{
	switch(chn)
	{
	case DMA_CHW_SPI:
		DMA->IF = (1 << DMA_IF_SPITX_Pos);
		break;
	
	case DMA_CHR_SPI:
		DMA->IF = (1 << DMA_IF_SPIRX_Pos);
		break;
	
	case DMA_CHR_ADC:
		DMA->IF = (1 << DMA_IF_ADC_Pos);
		break;
	
	case DMA_CHR_CAN:
		DMA->IF = (1 << DMA_IF_CAN_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: DMA_CH_INTStat()
* ����˵��:	DMA�ж�״̬��ѯ
* ��    ��: uint32_t chn			ָ��Ҫ���õ�ͨ������Чֵ��DMA_CHW_SPI��DMA_CHR_SPI��DMA_CHR_ADC��DMA_CHR_CAN
* ��    ��: uint32_t				1 ���ݰ������    0 ���ݰ���δ���
* ע������: ��
******************************************************************************************************************************************/
uint32_t DMA_CH_INTStat(uint32_t chn)
{
	uint32_t stat = 0;
	
	switch(chn)
	{
	case DMA_CHW_SPI:
		stat = (DMA->IF >> DMA_IF_SPITX_Pos) & 1;
		break;
	
	case DMA_CHR_SPI:
		stat = (DMA->IF >> DMA_IF_SPIRX_Pos) & 1;
		break;
	
	case DMA_CHR_ADC:
		stat = (DMA->IF >> DMA_IF_ADC_Pos)   & 1;
		break;
	
	case DMA_CHR_CAN:
		stat = (DMA->IF >> DMA_IF_CAN_Pos)   & 1;
		break;
	}
	
	return stat;
}
