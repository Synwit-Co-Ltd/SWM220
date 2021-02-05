#include "SWM220.h"


int main(void)
{	
	SystemInit();
	
	GPIO_Init(GPIOA, PIN5, 1, 0, 0, 0);			//�������LED
	
	TIMR_Init(TIMR0, TIMR_MODE_TIMER, SystemCoreClock/4, 1);	//ÿ0.25���Ӵ���һ���жϣ�ע�⣺���ڼĴ�����24λ�ģ�����������Լ16_000_000
	
	TIMR_Start(TIMR0);
	
	while(1==1)
	{
	}
}

void TIMR0_Handler(void)
{
	TIMR_INTClr(TIMR0);
	
	GPIO_InvBit(GPIOA, PIN5);	//��תLED����״̬
}
