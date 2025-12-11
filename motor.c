#include "motor.h"
#include <board.h>
#include "gv.h"

int MotorInit(void)
{
	OptoInit();
	rt_pin_mode(MOT_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(ROT_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(MOT_PIN, PIN_HIGH);	//初始化停止状态
	rt_pin_write(ROT_PIN, PIN_HIGH);	//初始化停止状态	



	return RT_EOK;
}
INIT_APP_EXPORT(MotorInit);

void MotorCtrl(MotorState state)
{
	switch (state)
	{
	case STOP:
	{
		rt_pin_write(MOT_PIN, PIN_HIGH);
		rt_pin_write(ROT_PIN, PIN_HIGH);
		break;
	}
	case START:
	{
		rt_pin_write(MOT_PIN, PIN_LOW);
		rt_pin_write(ROT_PIN, PIN_HIGH);
		break;
	}
	case ROTATE:
	{
		rt_pin_write(MOT_PIN, PIN_HIGH);
		rt_pin_write(ROT_PIN, PIN_LOW);
		break;
	}
	default:
		break;
	}
}

rt_uint8_t opto_flag = 0;

void Opto(void *args)
{
	int real = 0.1f*8388608/2.5f;
	int imag = 0.1f*8388608/2.5f;
	if(state_motor != 0)
	{
		//进入探测区域
		in_detection_zone = 1;
		//触发红外，添加边界值
		ad7190_pack(CH_AN1_AN2, real);
		ad7190_pack(CH_AN3_AN4, imag);
//		if(times_learning == TIMES_LEARNING)
//		{
//			//将学习模式转化为探测模式
//			statework = DETECTION;
//		}
//		else 
//		{
//			times_learning++;
//		}
	}

}

void OptoInit(void)
{	
	rt_pin_mode(LED1_PIN, PIN_MODE_OUTPUT);
	rt_pin_write(LED1_PIN, PIN_HIGH);
	rt_pin_mode(OPTO_PIN,PIN_MODE_INPUT);
	rt_pin_attach_irq(OPTO_PIN, PIN_IRQ_MODE_RISING,Opto,NULL);//红外挡住就触发中断
	rt_pin_irq_enable(OPTO_PIN,PIN_IRQ_ENABLE);

}
