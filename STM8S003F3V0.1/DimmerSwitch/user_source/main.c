/*

��оƬ:STM8S003F3
F = 20 pins; 3 = 8 Kbyte Flash
���뻷��:EWSTM8-2202-Autorun
IAR��V2.2
����:���⿪��

����ϵͳ:
����ϵͳ������,�ҹ��ܵ�һ�Ͳ����ö�ʱ��׼ȷ��ʱ��,��ż�ʱ����.
16MHz�����ⲿ��������(HSE)
16MHz�����ڲ�RC����(HSI)   Ŀǰ��Ϊϵͳʱ����

ʹ��һ��T4��Ϊ1MS��ʱ��,����������ȼ����,�����ϵͳ�в���Ҫ���ľ���

�����߼�:

���ⱸע:

*/

/*
��Դ����: �ܹ�20������
pin.
7     PA3       �������
8     PB5
9     PB4
10    PC3
11    PC4
12    PC5       TIM2_CH1
13    PC6       TIM1_CH1
14    PC7       TIM1_CH2
15    PD1       һ������SWIM���ؿ�
16    PD2       ��������
17    PD3       ��������
18    PD4
19    PD5
20    PD6       
*/
#include "stm8s.h"


#define TIM4_PERIOD       124

uint16_t CCR1_Val = 0;

#define CCR2_Val  ((uint16_t)1535)
#define CCR3_Val  ((uint16_t)1535)

    u32 u32_clk_freq;
__IO uint32_t TimingDelay = 0;
uint16_t ICValue1 =0, ICValue2 =0;
u16 opt_byte1,opt_byte2;
uint16_t learn_f_value[32];
u8  learn_f_flag;
uint16_t f_value;
BitStatus bit_status;


static void CLK_Config(void);
static void GPIO_Config(void);
void Delay(__IO uint32_t nTime);




/**
  * @brief  Configure TIM4 to generate an update interrupt each 1ms 
  * @param  None
  * @retval None
  */
static void TIM4_Config(void)
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

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while (TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
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
static void TIM1_Config(void)
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
static void TIM2_Config(void)
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



///////////////////////////////////////////////////////////////////////////////
//-���ܺ���ģ��
void calculate_learn_f(void)
{
  uint8_t i=0;
  uint16_t temp_vaule;
  
  temp_vaule = 0;
  for(i=0;i<32;i++)
  {
    temp_vaule += (learn_f_value[i] % 100);
  }
  
  f_value = (learn_f_value[0] / 100) * 100 + temp_vaule / 32;
}

    u8 KEY_Scan()
    {	 
              //-������IO״̬
          if(GPIO_ReadInputPin(GPIOB, GPIO_PIN_4)==RESET||GPIO_ReadInputPin(GPIOC, GPIO_PIN_4)==RESET)
            {
            Delay(100);
          if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_4)==RESET)  //SET or RESET
            return 1;
               else if(GPIO_ReadInputPin(GPIOC, GPIO_PIN_4)==RESET)
                 return 2;
            }
                else if(GPIO_ReadInputPin(GPIOB, GPIO_PIN_4)==SET||GPIO_ReadInputPin(GPIOC, GPIO_PIN_4)==SET)
                   return 0;
    }


                              



///////////////////////////////////////////////////////////////////////////////
void main( void )
{
    uint16_t i = 0;
    uint16_t flag = 1;
    uint8_t   level=0,j=0;
    u8 t;
   /* Init GPIO for LED  */
   GPIO_Config();   //-ǰ�ڿ���ͨ����������,ʹ��ʾ������ȷ��ʱ���׼

   /* CLK configuration --------------------------------------------*/
   CLK_Config();

/*   
   //-ѡ���ֽ� Ϊ�˶������Ÿ��ù���
   //-Define FLASH programming time
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    
    //- ѡ���ֽڿ���ͨ���������޸ĺ�,���ڿ��Բ����ڳ����в���
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    
   FLASH_EraseOptionByte(0x4803);
   FLASH_EraseOptionByte(0x4804);

   FLASH_ProgramOptionByte(0x4803, 0x01);
   FLASH_ProgramOptionByte(0x4804, 0xFE);
   
   FLASH_Lock(FLASH_MEMTYPE_DATA);

   opt_byte1 = FLASH_ReadOptionByte(0x4803);
//����ֵ��8λѡ���ֽں�8λ����
opt_byte2 = FLASH_ReadOptionByte(0x4804);
*/   
   TIM4_Config();  //-1msʱ��,���ȼ����
   
   TIM1_Config();   //-ʵ����׽����
   
   /* enable interrupts */
  enableInterrupts();
  
  //-������ɳ�ʼ��,Ȼ����ʱ�ȴ�һ��ʱ��,�����ʱ��������ѧϰ����,һ��ѧϰ�ɹ�,�Ͳ���ѧϰֱ���ϵ�
  Delay(11);
  
  //-ģ�Ⲷ׽��������
  while(flag)
  {
    //-ģ���������
     /*  Toggle the LEDs  */
     GPIOA->ODR |= (uint8_t)GPIO_PIN_3;   //-�����     
     /* Insert 100 ms delay */
    Delay(10);     
     GPIOA->ODR &= (uint8_t)~GPIO_PIN_3;    //-�����
     /* Insert 100 ms delay */
    Delay(10);
    if(learn_f_flag == 0x55)
    {
      calculate_learn_f();
      flag = 0;
    }
  }
  
  TIM2_Config();  //-���ú�������
  
   while (1)
   {
    //-ģ���������
     /*  Toggle the LEDs  */
     GPIOA->ODR |= (uint8_t)GPIO_PIN_3;   //-�����     
     /* Insert 100 ms delay */
    Delay(10);     
     GPIOA->ODR &= (uint8_t)~GPIO_PIN_3;    //-�����
     /* Insert 100 ms delay */
    Delay(10); 
     
//    t=KEY_Scan();		//�õ���ֵ
//	   if(t)
//	{						   
//	switch(t)
//	{				 
//              case 1:
//                 if(level <= 75)
//                    level+=5;
//                 else
//                    level = 0;
//     CCR1_Val = ((uint32_t)f_value * level) / 200;
//                 break;
//             case 2:
//               if(level >= 5)
//                   level-=5;
//               else
//                   level= 75;
//     CCR1_Val = ((uint32_t)f_value * level) / 200;
//               break;
//        }}
//        else  Delay(100);
  
               
    //-
    //-��ΧΪ0��73�ٷ�֮
    
    j++;
    if(j == 20)
    {
      if(level < 73)
        level++;
      else
        level = 0;
      Delay(1);
    }
    else if(j > 21)
      j = 0;
    
    CCR1_Val = ((uint32_t)f_value * level) / 200;
    
    }  } 
    //-������IO״̬
 //   if(GPIO_ReadInputPin(GPIOB, GPIO_PIN_4)==RESET)
//{
//      Delay(100);
//    if (GPIO_ReadInputPin(GPIOB, GPIO_PIN_4)==RESET)  //SET or RESET
//    {
//      if(level <= 75)
//       level+=5;
//      else
//        level = 0;
//    }
//}
//
//    if(GPIO_ReadInputPin(GPIOC, GPIO_PIN_4)==RESET)
//{
//      Delay(100);
//    if (GPIO_ReadInputPin(GPIOC, GPIO_PIN_4)==RESET)  //SET or RESET
//    {
//      if(level >= 5)
//       level-=5;
//      else
//        level = 75;
//    }
//}
    //-bit_status = GPIO_ReadInputPin(GPIOD, GPIO_PIN_3);
    //-if (bit_status == SET)  //SET or RESET
    //-{
    //-  
    //-}

    
    //-TIM2_SetCompare1(0x03E8);
    /*ICValue2 = TIM1_GetCounter();
    
    if((TIM1->SR1 & TIM1_FLAG_CC1) == TIM1_FLAG_CC1)
    {
      ICValue1 = TIM1_GetCapture1();
      TIM1_ClearFlag(TIM1_FLAG_CC1);
    }*/
  //}
//}









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
static void CLK_Config(void)
{
    ErrorStatus status = ERROR;
    
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
static void GPIO_Config(void)
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




#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





