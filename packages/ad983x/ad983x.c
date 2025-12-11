/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-03     Haojie       the first version
 */


#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_soft_spi.h"
#include "ad983x.h"

#define AD983X_FSYNC0_PIN      GET_PIN(B, 0)
#define AD983X_FSYNC1_PIN      GET_PIN(C, 5)
#define AD983X_FSYNC2_PIN      GET_PIN(C, 4)

/* MCLK frequency of dds */
#define F_MCLK              25000000  // 25MHz

/* Command Control Bits */

#define AD983X_B28          (1 << 13)
#define AD983X_HLB          (1 << 12)
#define AD983X_FSEL0        (0 << 11)
#define AD983X_FSEL1        (1 << 11)
#define AD983X_PSEL0        (0 << 10)
#define AD983X_PSEL1        (1 << 10)
#define AD983X_PIN_SW       (1 << 9)
#define AD983X_RESET        (1 << 8)
#define AD983X_SLEEP1       (1 << 7)
#define AD983X_SLEEP12      (1 << 6)
#define AD983X_OPBITEN      (1 << 5)
#define AD983X_SIGN_PIB     (1 << 4)
#define AD983X_DIV2         (1 << 3)
#define AD983X_MODE         (1 << 1)

#define AD983X_OUT_SINUS    ((0 << 5) | (0 << 1) | (0 << 3))
#define AD983X_OUT_TRIANGLE ((0 << 5) | (1 << 1) | (0 << 3))
#define AD983X_OUT_MSB      ((1 << 5) | (0 << 1) | (1 << 3))
#define AD983X_OUT_MSB2     ((1 << 5) | (0 << 1) | (0 << 3))


static struct rt_spi_device *spi_dev[3];


/***************************************************************************/ /**
 * @brief Writes the value to a register.
 *
 * @param -  regValue - The value to write to the register.
 *
 * @return  None.    
*******************************************************************************/

void AD983X_SetRegisterValue(rt_uint16_t regValue, rt_uint8_t chip)
{
    rt_uint8_t buf[2];

    buf[1] = (rt_uint8_t)(regValue & 0xFF);
    buf[0] = (rt_uint8_t)(regValue >> 8 & 0xFF);

    struct rt_spi_device *_spi_dev;

    _spi_dev = spi_dev[chip];

    if (_spi_dev)
    {
        rt_spi_send(_spi_dev, buf, 2);
    }
}

/***************************************************************************/ /**
 * @brief culculate the value of frequency registers.
 *
 * @param -  hz - The frequency of output.
 *
 * @return  FREQREG.
 *
 * @Note 2^28 = 268435456 
*******************************************************************************/
rt_uint32_t freq_2_reg(double hz)
{
    return 268435456.0 * hz / F_MCLK;
}

/***************************************************************************/ /**
 * @brief culculate the value of phase registers.
 *
 * @param -  rad - The phase of output, rad should less than 2*pi.
 *
 * @return  PHASEREG.
 *
 * @Note 2^12 = 4096 
*******************************************************************************/
rt_uint16_t rad_2_reg(double rad)
{
    return 4096.0 * rad / (2 * 3.1415926);
}

/***************************************************************************/ /**
 * @brief Writes to the frequency registers.
 *
 * @param -  reg - Frequence register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
void ad983x_set_frequency(rt_uint16_t reg, rt_uint32_t val, rt_uint8_t chip)
{
    rt_uint16_t freqHi = reg;
    rt_uint16_t freqLo = reg;

    freqHi |= (val & 0xFFFC000) >> 14;
    freqLo |= (val & 0x3FFF);

    AD983X_SetRegisterValue(freqLo, chip);
    AD983X_SetRegisterValue(freqHi, chip);
}
/***************************************************************************/ /**
 * @brief Writes to the phase registers.
 *
 * @param -  reg - Phase register to be written to.
 * @param -  val - The value to be written.
 *
 * @return  None.    
*******************************************************************************/
void ad983x_set_phase(rt_uint16_t reg, rt_uint16_t val, rt_uint8_t chip)
{
    rt_uint16_t phase = reg;
    phase |= val;
    AD983X_SetRegisterValue(phase, chip);
}

/**
  * @brief GPIO and SPI Initialization Function
  * @param None
  * @retval None
  */
static void ad983x_hw_init(void)
{
    //rt_pin_mode(AD983X_RST_PIN, PIN_MODE_OUTPUT);
    //rt_pin_write(AD983X_RST_PIN, PIN_LOW);
		rt_sw_spi_device_attach("spi3", "spi33", AD983X_FSYNC0_PIN);
		rt_sw_spi_device_attach("spi3", "spi34", AD983X_FSYNC1_PIN);
		rt_sw_spi_device_attach("spi3", "spi32", AD983X_FSYNC2_PIN);

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_2 | RT_SPI_MSB;
    cfg.max_hz = 10 * 1000 *1000;                           /* 10M */

    spi_dev[0] = (struct rt_spi_device *)rt_device_find("spi33");
    spi_dev[1] = (struct rt_spi_device *)rt_device_find("spi34");
    spi_dev[2] = (struct rt_spi_device *)rt_device_find("spi32");

    rt_spi_configure(spi_dev[0], &cfg);
    rt_spi_configure(spi_dev[1], &cfg);
    rt_spi_configure(spi_dev[2], &cfg);

}

/**
  * @brief ad983x Initialization Function
  * @param None
  * @retval None
  */
static int ad983x_init(void)
{
    rt_uint16_t val = 0;
    /* hardware initialization */
    ad983x_hw_init();
    /* ad9833配置为复位状态(不影响控制字写入) */
    val = AD983X_B28 | AD983X_RESET;
    AD983X_SetRegisterValue(val, 0);
    AD983X_SetRegisterValue(val, 1);
    AD983X_SetRegisterValue(val, 2);

    /* 设置REQ0为默认频率300KHz */
    ad983x_set_frequency(AD983X_REG_FREQ0, freq_2_reg(300000), 0);
    ad983x_set_frequency(AD983X_REG_FREQ0, freq_2_reg(300000), 1);
    ad983x_set_frequency(AD983X_REG_FREQ0, freq_2_reg(300000), 2);

    /* 设置PHASE0为默认相位 0rad */
    ad983x_set_phase(AD983X_REG_PHASE0, 0, 0);
    /* 设置PHASE0为默认相位 pi/2 rad */
    ad983x_set_phase(AD983X_REG_PHASE0, 0, 1);
	
    /* 设置PHASE0为默认相位 0rad */
    ad983x_set_phase(AD983X_REG_PHASE0, 4096 / 4, 2);

    /* 停止复位, 启动ad9833, 默认选择FREQ0, PHASE0, 正弦波输出*/
		val = AD983X_B28;
		/* 同时产生FSYNC，写入退出复位状态的指令，完成同步 */
		rt_pin_write(AD983X_FSYNC1_PIN, PIN_LOW);
    rt_pin_write(AD983X_FSYNC2_PIN, PIN_LOW);
    AD983X_SetRegisterValue(val, 0);
		rt_pin_write(AD983X_FSYNC1_PIN, PIN_HIGH);
		rt_pin_write(AD983X_FSYNC2_PIN, PIN_HIGH);

    //rt_pin_write(AD983X_RST_PIN, PIN_HIGH);
    return RT_EOK;
}


INIT_DEVICE_EXPORT(ad983x_init);  /* ad983x 外设驱动初始化 */

