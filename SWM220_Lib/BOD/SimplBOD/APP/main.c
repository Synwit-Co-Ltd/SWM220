#include "SWM220.h"


int main(void)
{	
	SystemInit();
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0, 0);			//�������LED
	
	SYS->BODCR = (1 << SYS_BODCR_INTEN_Pos) |
				 (0 << SYS_BODCR_INTLVL_Pos);	//2.7V�����ж�
	
	NVIC_EnableIRQ(BOD_IRQn);
		
	while(1==1)
	{
		GPIO_ClrBit(GPIOA, PIN5);
	}
}

void BOD_Handler(void)
{
	GPIO_SetBit(GPIOA, PIN5);
}
