#include "SWM220.h"


int main(void)
{	
	SystemInit();
	
	PORT_Init(PORTA, PIN4, PORTA_PIN4_TIMR2_OUT, 0);
	PORT_Init(PORTC, PIN4, PORTC_PIN4_TIMR2_OUT, 0);
	
	TIMR_Init(TIMR2, TIMR_MODE_TIMER, 10000, 0);	//�������ʱTIMRֻ��16λ��Ƶ��Ϊ24MHz/10000 = 2.4KHz
	
	TIMR_OC_Init(TIMR2, 2500, 0, 0);				//ռ�ձ�2500/10000 = 25%
	
	TIMR_Start(TIMR2);
	
	while(1==1)
	{
	}
}
