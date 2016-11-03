/**

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




uint16_t CCR1_Val = 0;

#define CCR2_Val  ((uint16_t)1535)
#define CCR3_Val  ((uint16_t)1535)

    

uint16_t ICValue1 =0, ICValue2 =0;
u16 opt_byte1,opt_byte2;
uint16_t learn_f_value[32];
u8  learn_f_flag;
uint16_t f_value;
BitStatus bit_status;



void Delay(__IO uint32_t nTime);


extern void TIM4_Config(void);
extern void TIM1_Config(void);
extern void GPIO_Config(void);
extern void CLK_Config(void);
extern void calculate_learn_f(void);
extern void TIM2_Config(void);








                



///////////////////////////////////////////////////////////////////////////////
void main( void )
{
    //-uint16_t i = 0;
    uint16_t flag = 1;
    uint8_t   level=0,j=0;
    //-u8 t;
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
    
    }  
}
///////////////////////////////////////////////////////////////////////////////





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





