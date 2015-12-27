/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "pwm_output.h"
#include "led.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


volatile int valid_rising_edge = 0;
int tim4_cnt = 0;
volatile int edge_count = 0;

/* I/O线中断，中断线为PA9 */
extern TIM_OCInitTypeDef  TIM_OCInitStructure;
static unsigned gTmp=0;
unsigned int gCount=2;
extern void Delay(u32 nTime);
void EXTI9_5_IRQHandler(void)
{
	int ReadValue=0;
	int i=0;
	static int action_count = 0;
	
	if(EXTI_GetITStatus(EXTI_Line9) != RESET) //确保是否产生了EXTI Line中断
	{
		EXTI_ClearITPendingBit(EXTI_Line9);     //清除中断标志位

		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) == 1)
		{
			ms_delay(10);
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_9) == 1)
			{
				edge_count++;
				if(edge_count == 1)
				{
					TIM_Cmd(TIM4, ENABLE); 
				}
				else if(edge_count == 2)
				{
					TIM_Cmd(TIM4, DISABLE);
					if(tim4_cnt < 5)
					{					
						valid_rising_edge = 1;
					}
					tim4_cnt = 0;
					edge_count = 0;
				}		
			}
		}

		if(valid_rising_edge)
		{
			valid_rising_edge = 0;
			action_count++;
			if(action_count == 1)
			{
				LED1(ON);
				TIM_Cmd(TIM3, DISABLE); 
				TIM_OCInitStructure.TIM_Pulse = 55;			//装载
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				TIM_Cmd(TIM3, ENABLE); 
			}
			else if(action_count == 2)
			{
				action_count = 0;
				LED1(OFF);
				TIM_Cmd(TIM3, DISABLE); 			
				TIM_OCInitStructure.TIM_Pulse = 50;			//卸载	
				TIM_OC1Init(TIM3, &TIM_OCInitStructure);
				TIM_Cmd(TIM3, ENABLE); 	
			}
		}
	}
}	


void TIM4_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)!=RESET)
	{      
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	 //清除中断标志
		
		tim4_cnt++;
		if(tim4_cnt > 25)				//> 500ms
		{
			tim4_cnt = 0;		
			edge_count = 0;
			TIM_Cmd(TIM4, DISABLE);	
		}
	}
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
