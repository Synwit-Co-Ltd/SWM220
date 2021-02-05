#ifndef __SWM220_WDT_H__
#define	__SWM220_WDT_H__

#define WDT_MODE_RESET			  0			//��������С��0ʱ������λ
#define WDT_MODE_INTERRUPT		  1			//��������С��LOAD/4ʱ�����ж�
#define WDT_MODE_INTERRUPT_RESET  2			//��������С��LOAD/4ʱ�����жϣ���С��0ʱ������λ

void WDT_Init(WDT_TypeDef * WDTx, uint32_t peroid, uint32_t mode);	//WDT���Ź���ʼ��
void WDT_Start(WDT_TypeDef * WDTx);			//����ָ��WDT����ʼ����ʱ
void WDT_Stop(WDT_TypeDef * WDTx);			//�ر�ָ��WDT��ֹͣ����ʱ

void WDT_Feed(WDT_TypeDef * WDTx);			//ι�������´�װ��ֵ��ʼ����ʱ

int32_t WDT_GetValue(WDT_TypeDef * WDTx);	//��ȡָ�����Ź���ʱ���ĵ�ǰ����ʱֵ


void WDT_INTClr(WDT_TypeDef * WDTx);		//�жϱ�־���
uint32_t WDT_INTStat(WDT_TypeDef * WDTx);	//�ж�״̬��ѯ
 
#endif //__SWM220_WDT_H__
