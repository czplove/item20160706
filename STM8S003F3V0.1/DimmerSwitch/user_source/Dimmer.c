/*
调光驱动:
应用层针对半波调节
前沿和后沿相位控制调光
Lcading Edgc and Trailing Edgc Phasc Control Dimming

前沿调光: 就是控制过零点后开始导通的时间点即前面不通,后面通

后沿调光: 前面导通后面关断,,不可误用于电感性负载
与前沿相位控制方式相反,当Vin过零后,T1或T2就立即导通,输出电压
Vout由零开始上升,当延迟一定相角<后,开关器件就切断电流,输出电压
Vout的波形如图六所示。
后沿相位控制调光器对呈容性的负载有极好的适应性,但如果将其应用于电感性负载时就产生了困难
*/

/*
需要实现函数的解耦
*/
#include "stm8s.h"



extern uint16_t learn_f_value[32];
extern u8  learn_f_flag;
extern uint16_t f_value;

extern void Delay(__IO uint32_t nTime);

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

#if 0
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
#endif















              








