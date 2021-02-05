#ifndef __SWM220_DMA_H__
#define __SWM220_DMA_H__


#define DMA_CHW_SPI		0	//��SPIͨ��
#define DMA_CHR_SPI		1	//дSPIͨ��
#define DMA_CHR_ADC		2	//��ADCͨ��
#define DMA_CHR_CAN		5	//��CANͨ��


void DMA_CH_Config(uint32_t chn, uint32_t ram_addr, uint32_t num_word, uint32_t int_en);	//DMAͨ������
void DMA_CH_Open(uint32_t chn);					//DMAͨ����
void DMA_CH_Close(uint32_t chn);				//DMAͨ���ر�

void DMA_CH_INTEn(uint32_t chn);				//DMA�ж�ʹ�ܣ����ݰ�����ɺ󴥷��ж�
void DMA_CH_INTDis(uint32_t chn);				//DMA�жϽ�ֹ�����ݰ�����ɺ󲻴����ж�
void DMA_CH_INTClr(uint32_t chn);				//DMA�жϱ�־���
uint32_t DMA_CH_INTStat(uint32_t chn);			//DMA�ж�״̬��ѯ��1 ���ݰ������    0 ���ݰ���δ���



#endif //__SWM220_DMA_H__
