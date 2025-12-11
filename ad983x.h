/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     Haojie       the first version
 */

#ifndef __AD983X_H_
#define __AD983X_H_

#include <rthw.h>
#include <rtthread.h>


/* Registers */

#define AD983X_REG_CMD      (0 << 14)
#define AD983X_REG_FREQ0    (1 << 14)
#define AD983X_REG_FREQ1    (2 << 14)
#define AD983X_REG_PHASE0   (6 << 13)
#define AD983X_REG_PHASE1   (7 << 13)

/* Writes to the frequency registers. */
void ad983x_set_frequency(rt_uint16_t reg, rt_uint32_t val ,rt_uint8_t chip);
/* Writes to the phase registers. */
void ad983x_set_phase(rt_uint16_t reg, rt_uint16_t val,rt_uint8_t chip);

#endif /* _AD9834_H */
