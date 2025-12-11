#ifndef __DETECTCTRL_H_
#define __DETECTCTRL_H_

#include <rtthread.h>
#include <board.h>
#include "motor.h"



#define GET_PIN(PORTx,PIN) (rt_base_t)((16 * ( ((rt_base_t)__GD32_PORT(PORTx) - (rt_base_t)GPIO_BASE)/(0x0400UL) )) + PIN)

#define TC_PIN GET_PIN(C, 1)
#define BJ_PIN GET_PIN(C, 2)

typedef enum
{
	JY = 0u,
	BJ,
	TC,
	TZ
}State_Detect;

int DetectCtrlInit(void);
void DetectCtrl(State_Detect state);

#endif
