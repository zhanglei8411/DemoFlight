/******************** (C) COPYRIGHT 2012 WildFire Team **************************
 * 文件名  ：main.c
 * 描述    ：PE5连接到key1，PE5配置为线中断模式，key1按下时，进入线中断处理函数，
 *           LED1状态取反。         
 * 实验平台：野火STM32开发板
 * 库版本  ：ST3.5.0
 *
 * 作者    ：wildfire team 
 * 论坛    ：http://www.amobbs.com/forum-1008-1.html
 * 淘宝    ：http://firestm32.taobao.com
**********************************************************************************/
#include "stm32f10x.h" 
#include "led.h"
#include "exti.h"
#include "pwm_output.h"

/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  ：无
 * 输出  ：无
 */
extern TIM_OCInitTypeDef  TIM_OCInitStructure;
extern GPIO_InitTypeDef GPIO_InitStructure;

int main(void)
{	


	/* config the led */
	LED_GPIO_Config();
	/* exti line config */
	EXTI_PA9_Config();
	
	//ms_delay(1000);
	TIM3_PWM_Init();
	TIM_Cmd(TIM3, ENABLE);
	
	TIM4_Init();
	
#if 0
	while(1)
	{

		ms_delay(500);
		
		TIM_OCInitStructure.TIM_Pulse = 55;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(500);
		TIM_OCInitStructure.TIM_Pulse = 50;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);

		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 200;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 195;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 190;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(500);
		TIM_OCInitStructure.TIM_Pulse = 195;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 200;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 205;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		ms_delay(20);
		TIM_OCInitStructure.TIM_Pulse = 210;
		TIM_OC1Init(TIM3, &TIM_OCInitStructure);

		for(i=205;i>=170;i--)
		{				
			TIM_OCInitStructure.TIM_Pulse = i;
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			ms_delay(1);
			
		}
		ms_delay(500);
		for(i=170;i<=205;i++)
		{				
			TIM_OCInitStructure.TIM_Pulse = i;
			TIM_OC1Init(TIM3, &TIM_OCInitStructure);
			ms_delay(1);
		}

	}
#endif

	

	
	/* wait interrupt */
	while(1)                            
	{
	}
}


/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
