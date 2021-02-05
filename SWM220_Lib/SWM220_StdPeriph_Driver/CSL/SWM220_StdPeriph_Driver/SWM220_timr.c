/****************************************************************************************************************************************** 
* �ļ�����: SWM220_timr.c
* ����˵��:	SWM220��Ƭ���ļ�����/��ʱ������������
* ����֧��:	http://www.synwit.com.cn/e/tool/gbook/?bid=1
* ע������:
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
#include "SWM220_timr.h"


/****************************************************************************************************************************************** 
* ��������: TIMR_Init()
* ����˵��:	TIMR��ʱ��/��������ʼ��
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������Чֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint32_t mode			TIMR_MODE_TIMER ��ʱ��ģʽ    TIMR_MODE_COUNTER ������ģʽ
*			uint32_t period			��ʱ/�������ڣ�24λ�����ֵԼ16000000
*			uint32_t int_en			�ж�ʹ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_Init(TIMR_TypeDef * TIMRx, uint32_t mode, uint32_t period, uint32_t int_en)
{
	SYS->CLKEN |= (0x01 << SYS_CLKEN_TIMR_Pos);
	
	TIMR_Stop(TIMRx);	//һЩ�ؼ��Ĵ���ֻ���ڶ�ʱ��ֹͣʱ����
	
	TIMRx->CTRL &= ~TIMR_CTRL_CLKSRC_Msk;
	TIMRx->CTRL |= mode << TIMR_CTRL_CLKSRC_Pos;
	
	TIMRx->LDVAL = period;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMR_INTClr(TIMR0);
		TIMRG->IE0 |= (1 << TIMRG_IE_IF_Pos);
		if(int_en) 
		{
			TIMRG->IM0 &= ~(1 << TIMRG_IM_IF_Pos);
			
			NVIC_EnableIRQ(TIMR0_IRQn);
		}
		else
		{
			TIMRG->IM0 |=  (1 << TIMRG_IM_IF_Pos);
			
			NVIC_DisableIRQ(TIMR0_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR1):
		TIMR_INTClr(TIMR1);
		TIMRG->IE1 |= (1 << TIMRG_IE_IF_Pos);
		if(int_en)
		{
			TIMRG->IM1 &= ~(1 << TIMRG_IM_IF_Pos);
			
			NVIC_EnableIRQ(TIMR1_IRQn);
		}
		else
		{
			TIMRG->IM1 |=  (1 << TIMRG_IM_IF_Pos);
			
			NVIC_DisableIRQ(TIMR1_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR2):
		TIMR_INTClr(TIMR2);
		TIMRG->IE2 |= (1 << TIMRG_IE_IF_Pos);
		if(int_en)
		{
			TIMRG->IM2 &= ~(1 << TIMRG_IM_IF_Pos);
			
			NVIC_EnableIRQ(TIMR2_IRQn);
		}
		else
		{
			TIMRG->IM2 |=  (1 << TIMRG_IM_IF_Pos);
			
			NVIC_DisableIRQ(TIMR2_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR3):
		TIMR_INTClr(TIMR3);
		TIMRG->IE3 |= (1 << TIMRG_IE_IF_Pos);
		if(int_en)
		{
			TIMRG->IM3 &= ~(1 << TIMRG_IM_IF_Pos);
			
			NVIC_EnableIRQ(TIMR3_IRQn);
		}
		else
		{
			TIMRG->IM3 |=  (1 << TIMRG_IM_IF_Pos);
			
			NVIC_DisableIRQ(TIMR3_IRQn);
		}
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_Start()
* ����˵��:	������ʱ�����ӳ�ʼֵ��ʼ��ʱ/����
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_Start(TIMR_TypeDef * TIMRx)
{
	TIMRx->CTRL |= (1 << TIMR_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_Stop()
* ����˵��:	ֹͣ��ʱ��
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_Stop(TIMR_TypeDef * TIMRx)
{
	TIMRx->CTRL &= ~(1 << TIMR_CTRL_EN_Pos);
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_Halt()
* ����˵��:	��ͣ��ʱ��������ֵ���ֲ���
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_Halt(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->HALT |= (0x01 << TIMRG_HALT_TIMR0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->HALT |= (0x01 << TIMRG_HALT_TIMR1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->HALT |= (0x01 << TIMRG_HALT_TIMR2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->HALT |= (0x01 << TIMRG_HALT_TIMR3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_Resume()
* ����˵��:	�ָ���ʱ��������ͣ����������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_Resume(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->HALT &= ~(0x01 << TIMRG_HALT_TIMR0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->HALT &= ~(0x01 << TIMRG_HALT_TIMR1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->HALT &= ~(0x01 << TIMRG_HALT_TIMR2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->HALT &= ~(0x01 << TIMRG_HALT_TIMR3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_SetPeriod()
* ����˵��:	���ö�ʱ/��������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint32_t period			��ʱ/��������
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_SetPeriod(TIMR_TypeDef * TIMRx, uint32_t period)
{
	TIMRx->LDVAL = period;
}

/****************************************************************************************************************************************** 
* ��������: TIMR_GetPeriod()
* ����˵��:	��ȡ��ʱ/��������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t				��ǰ��ʱ/��������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_GetPeriod(TIMR_TypeDef * TIMRx)
{
	return TIMRx->LDVAL; 
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_GetCurValue()
* ����˵��:	��ȡ��ǰ����ֵ
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t				��ǰ����ֵ
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_GetCurValue(TIMR_TypeDef * TIMRx)
{
	return TIMRx->CVAL;
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_INTEn()
* ����˵��:	ʹ���ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_INTEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 &= ~(1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 &= ~(1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 &= ~(1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 &= ~(1 << TIMRG_IM_IF_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_INTDis()
* ����˵��:	�����ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_INTDis(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 |= (1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 |= (1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 |= (1 << TIMRG_IM_IF_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 |= (1 << TIMRG_IM_IF_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_INTClr()
* ����˵��:	����жϱ�־
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_INTClr(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IF0 = (0x01 << TIMRG_IF_IF_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IF1 = (0x01 << TIMRG_IF_IF_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IF2 = (0x01 << TIMRG_IF_IF_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IF3 = (0x01 << TIMRG_IF_IF_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_INTStat()
* ����˵��:	��ȡ�ж�״̬
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t 				0 TIMRxδ�����ж�    1 TIMRx�������ж�
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_INTStat(TIMR_TypeDef * TIMRx)
{
	uint32_t stat = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		stat = ((TIMRG->IF0 >> TIMRG_IF_IF_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR1):
		stat = ((TIMRG->IF1 >> TIMRG_IF_IF_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR2):
		stat = ((TIMRG->IF2 >> TIMRG_IF_IF_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR3):
		stat = ((TIMRG->IF3 >> TIMRG_IF_IF_Pos) & 1);
		break;
	}
	
	return stat;
}

/****************************************************************************************************************************************** 
* ��������: TIMR_OC_Init()
* ����˵��:	����ȽϹ��ܳ�ʼ��
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint16_t match			����������ֵ�ݼ���matchʱ���������ƽ��ת
*			uint32_t match_int_en	����������ֵ�ݼ���matchʱ�Ƿ�����ж�
*			uint32_t init_lvl		��ʼ�����ƽ
* ��    ��: ��
* ע������: ʹ��Output Compare����ʱTIMRxֻ��16λ
******************************************************************************************************************************************/
void TIMR_OC_Init(TIMR_TypeDef * TIMRx, uint16_t match, uint32_t match_int_en, uint32_t init_lvl)
{
	TIMRx->CTRL &= ~(TIMR_CTRL_OC_MODE_Msk | TIMR_CTRL_OC_INILVL_Msk);
	TIMRx->CTRL |= (0 << TIMR_CTRL_OC_MODE_Pos) |
				   (init_lvl << TIMR_CTRL_OC_INILVL_Pos);
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->OCEN |= (1 << TIMRG_OCEN_TIMR0_Pos);
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR0_Pos);
		TIMRG->OCMAT0 = ((TIMR0->LDVAL) << 16) | match;
		TIMR_OC_INTClr(TIMR0);
		TIMRG->IE0 |= (1 << TIMRG_IE_OCMAT1_Pos);
		if(match_int_en) TIMRG->IM0 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		else			 TIMRG->IM0 |=  (1 << TIMRG_IM_OCMAT1_Pos);
		if(match_int_en | ((TIMRG->IM0 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR0_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR0_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->OCEN |= (1 << TIMRG_OCEN_TIMR1_Pos);
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR1_Pos);
		TIMRG->OCMAT1 =  ((TIMR1->LDVAL) << 16) | match;
		TIMR_OC_INTClr(TIMR1);
		TIMRG->IE1 |= (1 << TIMRG_IE_OCMAT1_Pos);
		if(match_int_en) TIMRG->IM1 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		else			 TIMRG->IM1 |=  (1 << TIMRG_IM_OCMAT1_Pos);
		if(match_int_en | ((TIMRG->IM1 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR1_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR1_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->OCEN |= (1 << TIMRG_OCEN_TIMR2_Pos);
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR2_Pos);
		TIMRG->OCMAT2 =  ((TIMR2->LDVAL) << 16) | match;
		TIMR_OC_INTClr(TIMR2);
		TIMRG->IE2 |= (1 << TIMRG_IE_OCMAT1_Pos);
		if(match_int_en) TIMRG->IM2 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		else			 TIMRG->IM2 |=  (1 << TIMRG_IM_OCMAT1_Pos);
		if(match_int_en | ((TIMRG->IM2 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR2_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR2_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->OCEN |= (1 << TIMRG_OCEN_TIMR3_Pos);
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR3_Pos);
		TIMRG->OCMAT3 =  ((TIMR3->LDVAL) << 16) | match;
		TIMR_OC_INTClr(TIMR3);
		TIMRG->IE3 |= (1 << TIMRG_IE_OCMAT1_Pos);
		if(match_int_en) TIMRG->IM3 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		else			 TIMRG->IM3 |=  (1 << TIMRG_IM_OCMAT1_Pos);
		if(match_int_en | ((TIMRG->IM3 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR3_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR3_IRQn);
		}
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_OC_OutputEn()
* ����˵��:	ʹ������ȽϹ��ܵĲ������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_OutputEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->OCMSK &= ~(1 << TIMRG_OCMSK_TIMR3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_OC_OutputDis()
* ����˵��:	��ֹ����ȽϹ��ܵĲ����������������ȽϹ������ű���level��ƽ
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint32_t level			��ֹ������κ��������ϱ��ֵĵ�ƽ
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_OutputDis(TIMR_TypeDef * TIMRx, uint32_t level)
{
	TIMRx->CTRL &= ~TIMR_CTRL_OC_INILVL_Msk;
	TIMRx->CTRL |= (level << TIMR_CTRL_OC_INILVL_Pos);
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->OCMSK |= (1 << TIMRG_OCMSK_TIMR0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->OCMSK |= (1 << TIMRG_OCMSK_TIMR1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->OCMSK |= (1 << TIMRG_OCMSK_TIMR2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->OCMSK |= (1 << TIMRG_OCMSK_TIMR3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_OC_SetMatch()
* ����˵��:	��������ȽϹ��ܵıȽ�ֵ
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint16_t match			����ȽϹ��ܵıȽ�ֵ
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_SetMatch(TIMR_TypeDef * TIMRx, uint16_t match)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->OCMAT0 = ((TIMR0->LDVAL) << 16) | match;
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->OCMAT1 = ((TIMR1->LDVAL) << 16) | match;
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->OCMAT2 = ((TIMR2->LDVAL) << 16) | match;
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->OCMAT3 = ((TIMR3->LDVAL) << 16) | match;
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_OC_GetMatch()
* ����˵��:	��ȡ����ȽϹ��ܵıȽ�ֵ
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint16_t				����ȽϹ��ܵıȽ�ֵ
* ע������: ��
******************************************************************************************************************************************/
uint16_t TIMR_OC_GetMatch(TIMR_TypeDef * TIMRx)
{
	uint16_t match = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		match = TIMRG->OCMAT0;
		break;
	
	case ((uint32_t)TIMR1):
		match = TIMRG->OCMAT1;
		break;
	
	case ((uint32_t)TIMR2):
		match = TIMRG->OCMAT2;
		break;
	
	case ((uint32_t)TIMR3):
		match = TIMRG->OCMAT3;
		break;
	}
	
	return match;
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_OC_INTEn()
* ����˵��:	ʹ������Ƚ��ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_INTEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 &= ~(1 << TIMRG_IM_OCMAT1_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_OC_INTDis()
* ����˵��:	��������Ƚ��ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_INTDis(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 |= (1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 |= (1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 |= (1 << TIMRG_IM_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 |= (1 << TIMRG_IM_OCMAT1_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_OC_INTClr()
* ����˵��:	�������Ƚ��жϱ�־
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_OC_INTClr(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IF0 = (0x01 << TIMRG_IF_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IF1 = (0x01 << TIMRG_IF_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IF2 = (0x01 << TIMRG_IF_OCMAT1_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IF3 = (0x01 << TIMRG_IF_OCMAT1_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_OC_INTStat()
* ����˵��:	��ȡ����Ƚ��ж�״̬
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t 				0 ����Ƚ�matchδ����   1 ����Ƚ�match����
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_OC_INTStat(TIMR_TypeDef * TIMRx)
{
	uint32_t stat = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		stat = ((TIMRG->IF0 >> TIMRG_IF_OCMAT1_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR1):
		stat = ((TIMRG->IF1 >> TIMRG_IF_OCMAT1_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR2):
		stat = ((TIMRG->IF2 >> TIMRG_IF_OCMAT1_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR3):
		stat = ((TIMRG->IF3 >> TIMRG_IF_OCMAT1_Pos) & 1);
		break;
	}
	
	return stat;
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_Init()
* ����˵��:	���벶���ܳ�ʼ��
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint32_t start_lvl		0 ��ʼ�����͵�ƽ����    1 ��ʼ�����ߵ�ƽ����
*			uint32_t captureH_int_en	�����ߵ�ƽ��������ж�ʹ��
*			uint32_t captureL_int_en	�����͵�ƽ��������ж�ʹ��
*			uint32_t captureOV_int_en	������/�͵�ƽ��������ж�ʹ��
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_Init(TIMR_TypeDef * TIMRx, uint32_t start_lvl, uint32_t captureH_int_en, uint32_t captureL_int_en, uint32_t captureOV_int_en)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->ICCR &= ~(TIMRG_ICCR_EN0_Msk | TIMRG_ICCR_POL0_Msk);
		TIMRG->ICCR |= (1 << TIMRG_ICCR_EN0_Pos) |
					   (start_lvl << TIMRG_ICCR_POL0_Pos);
		TIMRG->IE0 |= (1 << TIMRG_IE_ICH_Pos) |
					  (1 << TIMRG_IE_ICL_Pos) |
					  (1 << TIMRG_IE_ICOV_Pos);
		TIMR_IC_CaptureH_INTClr(TIMR0);
		if(captureH_int_en)  TIMRG->IM0 &= ~(1 << TIMRG_IM_ICH_Pos);
		else                 TIMRG->IM0 |=  (1 << TIMRG_IM_ICH_Pos);
		TIMR_IC_CaptureL_INTClr(TIMR0);
		if(captureL_int_en)  TIMRG->IM0 &= ~(1 << TIMRG_IM_ICL_Pos);
		else                 TIMRG->IM0 |=  (1 << TIMRG_IM_ICL_Pos);
		TIMR_IC_CaptureOV_INTClr(TIMR0);
		if(captureOV_int_en) TIMRG->IM0 &= ~(1 << TIMRG_IM_ICOV_Pos);
		else                 TIMRG->IM0 |=  (1 << TIMRG_IM_ICOV_Pos);
		if(captureH_int_en | captureL_int_en | captureOV_int_en | ((TIMRG->IM0 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR0_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR0_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->ICCR &= ~(TIMRG_ICCR_EN1_Msk | TIMRG_ICCR_POL1_Msk);
		TIMRG->ICCR |= (1 << TIMRG_ICCR_EN1_Pos) |
					   (start_lvl << TIMRG_ICCR_POL1_Pos);
		TIMRG->IE1 |= (1 << TIMRG_IE_ICH_Pos) |
					  (1 << TIMRG_IE_ICL_Pos) |
					  (1 << TIMRG_IE_ICOV_Pos);
		TIMR_IC_CaptureH_INTClr(TIMR1);
		if(captureH_int_en)  TIMRG->IM1 &= ~(1 << TIMRG_IM_ICH_Pos);
		else                 TIMRG->IM1 |=  (1 << TIMRG_IM_ICH_Pos);
		TIMR_IC_CaptureL_INTClr(TIMR1);
		if(captureL_int_en)  TIMRG->IM1 &= ~(1 << TIMRG_IM_ICL_Pos);
		else                 TIMRG->IM1 |=  (1 << TIMRG_IM_ICL_Pos);
		TIMR_IC_CaptureOV_INTClr(TIMR1);
		if(captureOV_int_en) TIMRG->IM1 &= ~(1 << TIMRG_IM_ICOV_Pos);
		else                 TIMRG->IM1 |=  (1 << TIMRG_IM_ICOV_Pos);
		if(captureH_int_en | captureL_int_en | captureOV_int_en | ((TIMRG->IM1 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR1_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR1_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->ICCR &= ~(TIMRG_ICCR_EN2_Msk | TIMRG_ICCR_POL2_Msk);
		TIMRG->ICCR |= (1 << TIMRG_ICCR_EN2_Pos) |
					   (start_lvl << TIMRG_ICCR_POL2_Pos);
		TIMRG->IE2 |= (1 << TIMRG_IE_ICH_Pos) |
					  (1 << TIMRG_IE_ICL_Pos) |
					  (1 << TIMRG_IE_ICOV_Pos);
		TIMR_IC_CaptureH_INTClr(TIMR2);
		if(captureH_int_en)  TIMRG->IM2 &= ~(1 << TIMRG_IM_ICH_Pos);
		else                 TIMRG->IM2 |=  (1 << TIMRG_IM_ICH_Pos);
		TIMR_IC_CaptureL_INTClr(TIMR2);
		if(captureL_int_en)  TIMRG->IM2 &= ~(1 << TIMRG_IM_ICL_Pos);
		else                 TIMRG->IM2 |=  (1 << TIMRG_IM_ICL_Pos);
		TIMR_IC_CaptureOV_INTClr(TIMR2);
		if(captureOV_int_en) TIMRG->IM2 &= ~(1 << TIMRG_IM_ICOV_Pos);
		else                 TIMRG->IM2 |=  (1 << TIMRG_IM_ICOV_Pos);
		if(captureH_int_en | captureL_int_en | captureOV_int_en | ((TIMRG->IM2 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR2_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR2_IRQn);
		}
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->ICCR &= ~(TIMRG_ICCR_EN3_Msk | TIMRG_ICCR_POL3_Msk);
		TIMRG->ICCR |= (1 << TIMRG_ICCR_EN3_Pos) |
					   (start_lvl << TIMRG_ICCR_POL3_Pos);
		TIMRG->IE3 |= (1 << TIMRG_IE_ICH_Pos) |
					  (1 << TIMRG_IE_ICL_Pos) |
					  (1 << TIMRG_IE_ICOV_Pos);
		TIMR_IC_CaptureH_INTClr(TIMR3);
		if(captureH_int_en)  TIMRG->IM3 &= ~(1 << TIMRG_IM_ICH_Pos);
		else                 TIMRG->IM3 |=  (1 << TIMRG_IM_ICH_Pos);
		TIMR_IC_CaptureL_INTClr(TIMR3);
		if(captureL_int_en)  TIMRG->IM3 &= ~(1 << TIMRG_IM_ICL_Pos);
		else                 TIMRG->IM3 |=  (1 << TIMRG_IM_ICL_Pos);
		TIMR_IC_CaptureOV_INTClr(TIMR3);
		if(captureOV_int_en) TIMRG->IM3 &= ~(1 << TIMRG_IM_ICOV_Pos);
		else                 TIMRG->IM3 |=  (1 << TIMRG_IM_ICOV_Pos);
		if(captureH_int_en | captureL_int_en | captureOV_int_en | ((TIMRG->IM3 & TIMRG_IM_IF_Msk) == 0)) 
		{
			NVIC_EnableIRQ(TIMR3_IRQn);
		}
		else
		{
			NVIC_DisableIRQ(TIMR3_IRQn);
		}
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_Start()
* ����˵��:	���벶����������ֻ�е�ϵͳ��ֻʹ��һ·���벶��ʱʹ�ô˺�����������ϵͳ��ʹ�ö�·���벶��ʱ��ʹ��TIMR_IC_Start_Multi()����
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ֻ�е�ϵͳ��ֻʹ��һ·���벶��ʱʹ�ô˺�����������ϵͳ��ʹ�ö�·���벶��ʱ��ʹ��TIMR_IC_Start_Multi()����
******************************************************************************************************************************************/
void TIMR_IC_Start(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->ICCR |= (1 << TIMRG_ICCR_GO0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->ICCR |= (1 << TIMRG_ICCR_GO1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->ICCR |= (1 << TIMRG_ICCR_GO2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->ICCR |= (1 << TIMRG_ICCR_GO3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_Start_Multi()
* ����˵��:	���벶����������ͬʱ������·
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
*			uint32_t timr0			�Ƿ�����TIMR0���벶���ܣ�0 ������   1 ����
*			uint32_t timr1			�Ƿ�����TIMR1���벶���ܣ�0 ������   1 ����
*			uint32_t timr2			�Ƿ�����TIMR2���벶���ܣ�0 ������   1 ����
*			uint32_t timr3			�Ƿ�����TIMR3���벶���ܣ�0 ������   1 ����
* ��    ��: ��
* ע������: ϵͳ��ʹ�ö�·���벶��ʱ��ʹ��TIMR_IC_Start_Multi()����
******************************************************************************************************************************************/
void TIMR_IC_Start_Multi(uint32_t timr0, uint32_t timr1, uint32_t timr2, uint32_t timr3)
{
	TIMRG->ICCR |= (timr0 << TIMRG_ICCR_GO0_Pos) | 
				   (timr1 << TIMRG_ICCR_GO1_Pos) | 
				   (timr2 << TIMRG_ICCR_GO2_Pos) | 
				   (timr3 << TIMRG_ICCR_GO3_Pos);
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_Stop()
* ����˵��:	���벶����ֹͣ
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_Stop(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->ICCR &= ~(1 << TIMRG_ICCR_GO0_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->ICCR &= ~(1 << TIMRG_ICCR_GO1_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->ICCR &= ~(1 << TIMRG_ICCR_GO2_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->ICCR &= ~(1 << TIMRG_ICCR_GO3_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_GetCaptureH()
* ����˵��:	��ȡ�ߵ�ƽ���Ȳ������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t				�ߵ�ƽ���Ȳ������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_IC_GetCaptureH(TIMR_TypeDef * TIMRx)
{
	uint32_t capture = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		capture = TIMRG->ICVAL0H;
		break;
	
	case ((uint32_t)TIMR1):
		capture = TIMRG->ICVAL1H;
		break;
	
	case ((uint32_t)TIMR2):
		capture = TIMRG->ICVAL2H;
		break;
	
	case ((uint32_t)TIMR3):
		capture = TIMRG->ICVAL3H;
		break;
	}
	
	return capture;
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_GetCaptureL()
* ����˵��:	��ȡ�͵�ƽ���Ȳ������
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t				�͵�ƽ���Ȳ������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_IC_GetCaptureL(TIMR_TypeDef * TIMRx)
{
	uint32_t capture = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		capture = TIMRG->ICVAL0L;
		break;
	
	case ((uint32_t)TIMR1):
		capture = TIMRG->ICVAL1L;
		break;
	
	case ((uint32_t)TIMR2):
		capture = TIMRG->ICVAL2L;
		break;
	
	case ((uint32_t)TIMR3):
		capture = TIMRG->ICVAL3L;
		break;
	}
	
	return capture;
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureH_INTEn()
* ����˵��:	ʹ�����벶��ߵ�ƽ���Ȳ�������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureH_INTEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 &= ~(1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 &= ~(1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 &= ~(1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 &= ~(1 << TIMRG_IM_ICH_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureH_INTDis()
* ����˵��:	�������벶��ߵ�ƽ���Ȳ�������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureH_INTDis(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 |= (1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 |= (1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 |= (1 << TIMRG_IM_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 |= (1 << TIMRG_IM_ICH_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureH_INTClr()
* ����˵��:	������벶��ߵ�ƽ���Ȳ�������жϱ�־
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureH_INTClr(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IF0 = (0x01 << TIMRG_IF_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IF1 = (0x01 << TIMRG_IF_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IF2 = (0x01 << TIMRG_IF_ICH_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IF3 = (0x01 << TIMRG_IF_ICH_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureH_INTStat()
* ����˵��:	��ȡ���벶��ߵ�ƽ���Ȳ�������ж�״̬
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t 				0 �ߵ�ƽ���Ȳ���δ���    1 �ߵ�ƽ���Ȳ������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_IC_CaptureH_INTStat(TIMR_TypeDef * TIMRx)
{
	uint32_t stat = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		stat = ((TIMRG->IF0 >> TIMRG_IF_ICH_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR1):
		stat = ((TIMRG->IF1 >> TIMRG_IF_ICH_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR2):
		stat = ((TIMRG->IF2 >> TIMRG_IF_ICH_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR3):
		stat = ((TIMRG->IF3 >> TIMRG_IF_ICH_Pos) & 1);
		break;
	}
	
	return stat;
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureL_INTEn()
* ����˵��:	ʹ�����벶��͵�ƽ���Ȳ�������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureL_INTEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 &= ~(1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 &= ~(1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 &= ~(1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 &= ~(1 << TIMRG_IM_ICL_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureL_INTDis()
* ����˵��:	�������벶��͵�ƽ���Ȳ�������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureL_INTDis(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 |= (1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 |= (1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 |= (1 << TIMRG_IM_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 |= (1 << TIMRG_IM_ICL_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureL_INTClr()
* ����˵��:	������벶��͵�ƽ���Ȳ�������жϱ�־
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureL_INTClr(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IF0 = (0x01 << TIMRG_IF_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IF1 = (0x01 << TIMRG_IF_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IF2 = (0x01 << TIMRG_IF_ICL_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IF3 = (0x01 << TIMRG_IF_ICL_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureL_INTStat()
* ����˵��:	��ȡ���벶��͵�ƽ���Ȳ�������ж�״̬
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t 				0 �͵�ƽ���Ȳ���δ���    1 �͵�ƽ���Ȳ������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_IC_CaptureL_INTStat(TIMR_TypeDef * TIMRx)
{
	uint32_t stat = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		stat = ((TIMRG->IF0 >> TIMRG_IF_ICL_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR1):
		stat = ((TIMRG->IF1 >> TIMRG_IF_ICL_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR2):
		stat = ((TIMRG->IF2 >> TIMRG_IF_ICL_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR3):
		stat = ((TIMRG->IF3 >> TIMRG_IF_ICL_Pos) & 1);
		break;
	}
	
	return stat;
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureOV_INTEn()
* ����˵��:	ʹ�����벶������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureOV_INTEn(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 &= ~(1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 &= ~(1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 &= ~(1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 &= ~(1 << TIMRG_IM_ICOV_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureOV_INTDis()
* ����˵��:	�������벶������ж�
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureOV_INTDis(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IM0 |= (1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IM1 |= (1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IM2 |= (1 << TIMRG_IM_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IM3 |= (1 << TIMRG_IM_ICOV_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������:	TIMR_IC_CaptureOV_INTClr()
* ����˵��:	������벶������жϱ�־
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: ��
* ע������: ��
******************************************************************************************************************************************/
void TIMR_IC_CaptureOV_INTClr(TIMR_TypeDef * TIMRx)
{
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		TIMRG->IF0 = (0x01 << TIMRG_IF_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR1):
		TIMRG->IF1 = (0x01 << TIMRG_IF_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR2):
		TIMRG->IF2 = (0x01 << TIMRG_IF_ICOV_Pos);
		break;
	
	case ((uint32_t)TIMR3):
		TIMRG->IF3 = (0x01 << TIMRG_IF_ICOV_Pos);
		break;
	}
}

/****************************************************************************************************************************************** 
* ��������: TIMR_IC_CaptureOV_INTStat()
* ����˵��:	��ȡ���벶������ж�״̬
* ��    ��: TIMR_TypeDef * TIMRx	ָ��Ҫ�����õĶ�ʱ������ȡֵ����TIMR0��TIMR1��TIMR2��TIMR3
* ��    ��: uint32_t 				0 δ�������    1 �������
* ע������: ��
******************************************************************************************************************************************/
uint32_t TIMR_IC_CaptureOV_INTStat(TIMR_TypeDef * TIMRx)
{
	uint32_t stat = 0;
	
	switch((uint32_t)TIMRx)
	{
	case ((uint32_t)TIMR0):
		stat = ((TIMRG->IF0 >> TIMRG_IF_ICOV_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR1):
		stat = ((TIMRG->IF1 >> TIMRG_IF_ICOV_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR2):
		stat = ((TIMRG->IF2 >> TIMRG_IF_ICOV_Pos) & 1);
		break;
	
	case ((uint32_t)TIMR3):
		stat = ((TIMRG->IF3 >> TIMRG_IF_ICOV_Pos) & 1);
		break;
	}
	
	return stat;
}
