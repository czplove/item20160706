/**
����Ӳ���ĳ�ʼ��
*/

/*
��Ҫʵ�ֺ����Ľ���
*/
#include "stm8s.h"



#define TIM4_PERIOD       124


extern uint16_t f_value;
extern uint16_t CCR1_Val;


u32 u32_clk_freq;



/**
  * @brief  Configure TIM4 to generate an update interrupt each 1ms 
  * @param  None
  * @retval None
  */
void TIM4_Config(void)
{
  /* TIM4 configuration:
   - TIM4CLK is set to 16 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
   clock used is 16 MHz / 128 = 125 000 Hz
  - With 125 000 Hz we can generate time base:
      max time base is 2.048 ms if TIM4_PERIOD = 255 --> (255 + 1) / 125000 = 2.048 ms
      min time base is 0.016 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 125000 = 0.016 ms
  - In this example we need to generate a time base equal to 1 ms
   so TIM4_PERIOD = (0.001 * 125000 - 1) = 124 */

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
  /* Clear TIM4 update flag */
  TIM4_ClearFlag(TIM4_FLAG_UPDATE);
  /* Enable update interrupt */
  TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
  

  /* Enable TIM4 */
  TIM4_Cmd(ENABLE);
}


///////////////////////////////////////////////////////////////////////////////

/*
16MHz ��Ƶ��һ������0.0625uS
������Ҫ�Ѷ�ʱ��1�ķֱ��ʷŵ�1uS ��ô������16��Ƶ֮��Ϳ�����

*/
/**
  * @brief  Configure TIM1 to to capture the internal clock source (LSI)
  * @param  None
  * @retval None
  */
void TIM1_Config(void)
{
  TIM1_DeInit();  //-��ʱ��(fMASTER)��Ϊ��ʱ���ṩʱ�ӻ�׼
  
  TIM1_TimeBaseInit(0x000F, TIM1_COUNTERMODE_UP, 0xFFFF, 0x00); //-0��Ƶʱ ��������ʱ��Ƶ�� = ��ʱ��(fMASTER)
  
  
  
  //-���벶׽
  TIM1_ICInit( TIM1_CHANNEL_1, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
               TIM1_ICPSC_DIV1, 0x0);
  
  
  TIM1_SelectInputTrigger(TIM1_TIXEXTERNALCLK1SOURCE_TI2);

  
  
  
  
  
  
  //-���
  /*
  TIM1_OC2Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_DISABLE, CCR2_Val,
               TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET, 
               TIM1_OCNIDLESTATE_RESET);
  
  TIM1_OC3Init(TIM1_OCMODE_PWM2, TIM1_OUTPUTSTATE_ENABLE, TIM1_OUTPUTNSTATE_ENABLE,
               CCR3_Val, TIM1_OCPOLARITY_LOW, TIM1_OCNPOLARITY_HIGH, TIM1_OCIDLESTATE_SET,
               TIM1_OCNIDLESTATE_RESET);
  */
  /* Enable TIM1 */
  TIM1_Cmd(ENABLE);

  /* Clear CC1 Flag*/
  TIM1_ClearFlag(TIM1_FLAG_CC1);
  
  //-TIM1_CtrlPWMOutputs(ENABLE);
  
  
  TIM1_CCxCmd(TIM1_CHANNEL_1, ENABLE);
  
  TIM1_ITConfig(TIM1_IT_CC1, ENABLE);
  
  /* wait a capture on CC1 */
  //-while((TIM1->SR1 & TIM1_FLAG_CC1) != TIM1_FLAG_CC1);
  /* Get CCR1 value*/
  //-ICValue1 = TIM1_GetCapture1();
  //-TIM1_ClearFlag(TIM1_FLAG_CC1);
}

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/*
ʹ���������̶����ڵ�PWM����,�м���������Ҫ���.
1.��̬���޸���Ч����   TIM2_SetCompare1(0x0000);
2.����ͬ�����  TIM2_SetCounter(0x0000);


��PWMģʽ(ģʽ1��ģʽ2)�£� TIM1_CNT��TIM1_CCRiʼ���ڽ��бȽϣ� (���ݼ������ļ���
����)��ȷ���Ƿ����TIM1_CCRi��TIM1_CNT����TIM1_CNT��TIM1_CCRi��
*/
/**
  * @brief  Configure TIM2 peripheral in PWM mode
  * @param  None
  * @retval None
  */
void TIM2_Config(void)
{
  /* Time base configuration */
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, f_value/2);  //-ʱ����Ԫ��1uS,������ڿ�����65535;

  /* PWM1 Mode configuration: Channel1 */ 
  TIM2_OC1Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,CCR1_Val, TIM2_OCPOLARITY_HIGH);
  TIM2_OC1PreloadConfig(ENABLE);

  /* PWM1 Mode configuration: Channel2 */ 
  TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,CCR1_Val, TIM2_OCPOLARITY_HIGH);
  TIM2_OC2PreloadConfig(ENABLE);

  /* PWM1 Mode configuration: Channel3 */         
  //-TIM2_OC3Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE,CCR3_Val, TIM2_OCPOLARITY_HIGH);
  //-TIM2_OC3PreloadConfig(ENABLE);

  TIM2_ARRPreloadConfig(ENABLE);

  /* TIM2 enable counter */
  TIM2_Cmd(ENABLE);
}
///////////////////////////////////////////////////////////////////////////////



/*
����ʱ����ʱ��ԴĬ��ΪHSI RCʱ�ӵ�8��Ƶ����fHSI/8
һ���ⲿʱ��Դ�ȶ�,�ɽ����л�.
CPUʱ��(fCPU)����ʱ��(fMASTER)��Ƶ��������Ƶ������ʱ�ӷ�Ƶ�Ĵ���(CLK_CKDIVR)�е�λ
CPUDIV[2:0]��������7����Ƶ���ӿɹ�ѡ��(1��128�У� 2����)��
*/
/**
  * @brief  Configure system clock to use HSE as source clock and to enable the 
  *         Clock Security System (CSS)  
  * @param  None
  * @retval None
  */
void CLK_Config(void)
{
    //-ErrorStatus status = ERROR;
    
    CLK_DeInit(); //-�ָ���ص�ʱ�ӼĴ�����Ĭ��ֵ
    
    CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
    
    CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
    

u32_clk_freq = CLK_GetClockFreq();
    //-status = CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO, CLK_SOURCE_HSE, DISABLE,
    //-                               CLK_CURRENTCLOCKSTATE_DISABLE);

    /*Enable CSS interrupt */
    //-CLK_ITConfig(CLK_IT_CSSD, ENABLE);
    
    /* Enable CCS */
    //-CLK_ClockSecuritySystemEnable();

    /* Output Fcpu on CLK_CCO pin */
    //-CLK_CCOConfig(CLK_OUTPUT_MASTER);
    
    //-enableInterrupts();
}

/**
  * @brief  Configure GPIO for LEDs available on the evaluation board
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
    /* Configure the GPIO_LED pin */
    GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_HIGH_FAST);
    //-STM_EVAL_LEDInit(LED2);
    
    //-����IO �������ж�
    GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_PU_NO_IT);

    //-GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);
    GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);

}
















