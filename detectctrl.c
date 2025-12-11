#include "detectctrl.h"



int DetectCtrlInit(void)
{
	rt_pin_mode(BJ_PIN, PIN_MODE_OUTPUT);
	rt_pin_mode(TC_PIN, PIN_MODE_OUTPUT);
	return 0;
}
INIT_APP_EXPORT(DetectCtrlInit);

void DetectCtrl(State_Detect state)
{
	switch (state)
	{
		case TC:
		{
			rt_pin_write(TC_PIN, PIN_HIGH);
			break;
		}
		case JY:
		{
			break;
		}
		case BJ:
		{
			rt_pin_write(BJ_PIN, PIN_HIGH);
			break;
		}
		case TZ:
		{
			MotorCtrl(STOP);
			break;
		}
	}
}
