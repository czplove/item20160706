/*

主芯片:STM8S003F3
F = 20 pins; 3 = 8 Kbyte Flash
编译环境:EWSTM8-2202-Autorun
IAR的V2.2
功能:调光开关

基本系统:
由于系统本身下,且功能单一就不采用定时器准确记时了,大概记时即可.
16MHz高速外部晶体振荡器(HSE)
16MHz高速内部RC振荡器(HSI)   目前作为系统时钟用

使用一个T4作为1MS定时器,但是这个优先级最低,在这个系统中不需要他的精度

调光逻辑:

特殊备注:

*/

/*
资源分配: 总共20个引脚
pin.
7     PA3       可以输出
8     PB5
9     PB4
10    PC3
11    PC4
12    PC5       TIM2_CH1
13    PC6       TIM1_CH1
14    PC7       TIM1_CH2
15    PD1       一般用作SWIM下载口
16    PD2       可以输入
17    PD3       可以输入
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
16MHz 的频率一次脉冲0.0625uS
现在需要把定时器1的分辨率放到1uS 那么进行了16分频之后就可以了

*/
/**
  * @brief  Configure TIM1 to to capture the internal clock source (LSI)
  * @param  None
  * @retval None
  */
static void TIM1_Config(void)
{
  TIM1_DeInit();  //-主时钟(fMASTER)可为计时器提供时钟基准
  
  TIM1_TimeBaseInit(0x000F, TIM1_COUNTERMODE_UP, 0xFFFF, 0x00); //-0分频时 计数器的时钟频率 = 主时钟(fMASTER)
  
  
  
  //-输入捕捉
  TIM1_ICInit( TIM1_CHANNEL_1, TIM1_ICPOLARITY_RISING, TIM1_ICSELECTION_DIRECTTI,
               TIM1_ICPSC_DIV1, 0x0);
  
  
  TIM1_SelectInputTrigger(TIM1_TIXEXTERNALCLK1SOURCE_TI2);

  
  
  
  
  
  
  //-输出
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
使用这个输出固定周期的PWM波形,有几个问题需要解决.
1.动态的修改有效脉宽   TIM2_SetCompare1(0x0000);
2.快速同步零点  TIM2_SetCounter(0x0000);


在PWM模式(模式1或模式2)下， TIM1_CNT和TIM1_CCRi始终在进行比较， (依据计数器的计数
方向)以确定是否符合TIM1_CCRi≤TIM1_CNT或者TIM1_CNT≤TIM1_CCRi。
*/
/**
  * @brief  Configure TIM2 peripheral in PWM mode
  * @param  None
  * @retval None
  */
static void TIM2_Config(void)
{
  /* Time base configuration */
  TIM2_TimeBaseInit(TIM2_PRESCALER_16, f_value/2);  //-时基单元是1uS,最大周期可以是65535;

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
//-功能函数模块
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
              //-读输入IO状态
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
   GPIO_Config();   //-前期可以通过这个的输出,使用示波器来确定时间基准

   /* CLK configuration --------------------------------------------*/
   CLK_Config();

/*   
   //-选项字节 为了定义引脚复用功能
   //-Define FLASH programming time
    FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
    
    //- 选项字节可以通过下载器修改好,现在可以不用在程序中操作
    FLASH_Unlock(FLASH_MEMTYPE_DATA);
    
   FLASH_EraseOptionByte(0x4803);
   FLASH_EraseOptionByte(0x4804);

   FLASH_ProgramOptionByte(0x4803, 0x01);
   FLASH_ProgramOptionByte(0x4804, 0xFE);
   
   FLASH_Lock(FLASH_MEMTYPE_DATA);

   opt_byte1 = FLASH_ReadOptionByte(0x4803);
//返回值：8位选项字节和8位补码
opt_byte2 = FLASH_ReadOptionByte(0x4804);
*/   
   TIM4_Config();  //-1ms时钟,优先级最低
   
   TIM1_Config();   //-实现扑捉功能
   
   /* enable interrupts */
  enableInterrupts();
  
  //-首先完成初始化,然后延时等待一段时间,在这个时间内用于学习周期,一旦学习成功,就不再学习直到断电
  Delay(11);
  
  //-模拟捕捉计算周期
  while(flag)
  {
    //-模拟输出脉宽
     /*  Toggle the LEDs  */
     GPIOA->ODR |= (uint8_t)GPIO_PIN_3;   //-输出高     
     /* Insert 100 ms delay */
    Delay(10);     
     GPIOA->ODR &= (uint8_t)~GPIO_PIN_3;    //-输出低
     /* Insert 100 ms delay */
    Delay(10);
    if(learn_f_flag == 0x55)
    {
      calculate_learn_f();
      flag = 0;
    }
  }
  
  TIM2_Config();  //-设置好了周期
  
   while (1)
   {
    //-模拟输出脉宽
     /*  Toggle the LEDs  */
     GPIOA->ODR |= (uint8_t)GPIO_PIN_3;   //-输出高     
     /* Insert 100 ms delay */
    Delay(10);     
     GPIOA->ODR &= (uint8_t)~GPIO_PIN_3;    //-输出低
     /* Insert 100 ms delay */
    Delay(10); 
     
//    t=KEY_Scan();		//得到键值
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
    //-范围为0到73百分之
    
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
    //-读输入IO状态
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
启动时，主时钟源默认为HSI RC时钟的8分频，即fHSI/8
一旦外部时钟源稳定,可进行切换.
CPU时钟(fCPU)由主时钟(fMASTER)分频而来，分频因子由时钟分频寄存器(CLK_CKDIVR)中的位
CPUDIV[2:0]决定。共7个分频因子可供选择(1至128中， 2的幂)。
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
    
    CLK_DeInit(); //-恢复相关的时钟寄存器到默认值
    
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
    
    //-输入IO 上拉无中断
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





