/*
��������:
Ӧ�ò���԰벨����
ǰ�غͺ�����λ���Ƶ���
Lcading Edgc and Trailing Edgc Phasc Control Dimming

ǰ�ص���: ���ǿ��ƹ�����ʼ��ͨ��ʱ��㼴ǰ�治ͨ,����ͨ

���ص���: ǰ�浼ͨ����ض�,,���������ڵ���Ը���
��ǰ����λ���Ʒ�ʽ�෴,��Vin�����,T1��T2��������ͨ,�����ѹ
Vout���㿪ʼ����,���ӳ�һ�����<��,�����������жϵ���,�����ѹ
Vout�Ĳ�����ͼ����ʾ��
������λ���Ƶ������Գ����Եĸ����м��õ���Ӧ��,���������Ӧ���ڵ���Ը���ʱ�Ͳ���������
*/

/*
��Ҫʵ�ֺ����Ľ���
*/
#include "stm8s.h"



extern uint16_t learn_f_value[32];
extern u8  learn_f_flag;
extern uint16_t f_value;

extern void Delay(__IO uint32_t nTime);

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

#if 0
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
#endif















              








