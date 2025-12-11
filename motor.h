#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <rtthread.h>
#include "ad7190.h"

#define MOT_PIN  GET_PIN(A, 0)
#define ROT_PIN  GET_PIN(C, 3)
#define OPTO_PIN GET_PIN(B, 3)

#define PIN_IRQ_MODE PIN_IRQ_MODE_HIGH_LEVEL
#define LED1_PIN GET_PIN(A,1)

typedef enum
{
	STOP = 0U,
	START,
	ROTATE
} MotorState;

extern rt_uint8_t opto_flag;

int MotorInit(void);
void MotorCtrl(MotorState state);
void OptoInit(void);

#endif
