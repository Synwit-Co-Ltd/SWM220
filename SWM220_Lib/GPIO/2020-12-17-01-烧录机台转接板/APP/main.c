#include "SWM220.h"
  
  
#define OK_PROG  		    GPIOC->DAT.DAT_13
#define NG_PROG 			  GPIOA->DAT.DAT_6
#define BUSY_PROG  		  GPIOC->DAT.DAT_15
#define Start_PROG  		GPIOA->DAT.DAT_7

#define OK_CTRL  		    GPIOB->DAT.DAT_8
#define NG_CTRL 			  GPIOC->DAT.DAT_5
#define BUSY_CTRL  		  GPIOC->DAT.DAT_6
#define Start_CRTL  		GPIOA->DAT.DAT_9



uint16_t T_Start_Pro;
uint16_t T_Start_Send;
void TIMR0_Handler(void)
{
	TIMR_INTClr(TIMR0);
  
  
  
  if(T_Start_Pro)         T_Start_Pro--;
  if(T_Start_Send)        T_Start_Send--;

}

uint8_t Flag_Start_Send = 0;
void SendStart(void)
{
  if(T_Start_Send == 0 && Flag_Start_Send < 2)
  {
    T_Start_Send = 100;
    Flag_Start_Send++;
    
    Start_PROG = 0;
    for(uint32_t i = 0;  i < 4000 * 50; i++);
    Start_PROG = 1;
  }
  else  if(Flag_Start_Send == 2)
  {
    Flag_Start_Send++;
    GPIO_Init(GPIOA, PIN7 , 0, 0, 0, 0);       //A7  Start 
  }
}




int main(void)
{	
	SystemInit();
	
  TIMR_Init(TIMR0, TIMR_MODE_TIMER, SystemCoreClock / 1000, 1);	//每0.25秒钟触发一次中断；注意：周期寄存器是24位的，最大计数个数约16_000_000
	TIMR_Start(TIMR0);
  
  //接编程器
  //除了Start 其他都是输入
  GPIO_Init(GPIOC, PIN13, 0, 0, 0, 0);      //C13   OK
  GPIO_Init(GPIOA, PIN6 , 0, 0, 0, 0);       //A6    NG
  GPIO_Init(GPIOC, PIN15, 0, 0, 0, 0);      //C15   BUSY       
//  GPIO_Init(GPIOA, PIN7 , 1, 0, 0, 0);       //A7  Start 
  
  //接烧录机台
  //除了Start 其他都是输入
  GPIO_Init(GPIOB, PIN8 , 1, 0, 0, 0);      //B8   OK
  GPIO_Init(GPIOC, PIN5 , 1, 0, 0, 0);       //C5    NG
  GPIO_Init(GPIOC, PIN6 , 1, 0, 0, 0);      //C6   BUSY       
  GPIO_Init(GPIOA, PIN9 , 0, 0, 1, 0);       //A9  Start
  
  GPIO_Init(GPIOA, PIN7 , 0, 0, 0, 0);       //A7  Start 
  
  //初始状态
  Start_PROG = 1;
  
  
	while(1==1)
	{
		OK_CTRL     = !OK_PROG;
    BUSY_CTRL   = !BUSY_PROG;
    NG_CTRL     = !NG_PROG;
    
    SendStart();
    if(Start_CRTL == 1  && T_Start_Pro == 0)
    {
      T_Start_Pro = 200;
      
      GPIO_Init(GPIOA, PIN7 , 1, 0, 0, 0);       //A7  Start 
      Flag_Start_Send = 0;
    }
	}
}



