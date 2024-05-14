/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_bringup.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/







#include <syslog.h>

#include "tm4c123g-launchpad.h"

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>
#include <nuttx/sensors/bmp280.h>
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/sensors/qencoder.h>
#include <nuttx/sensors/as5048b.h>
#include <arch/board/board.h>
#include <nuttx/fs/fs.h>

#include <nuttx/timers/pwm.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "tiva_i2c.h"
#include <nuttx/serial/serial.h>
#include<nuttx/can/can.h>
#include <nuttx/ioexpander/gpio.h>

#include</home/astrekdev1/nuttxspace/nuttx/arch/arm/src/tiva/tm4c/tm4c_gpio.h>


#ifdef HAVE_USERLED_DRIVER
#  include <nuttx/leds/userled.h>
#endif


#define BOARD_NGPIOOUT 1
#define BOARD_NGPIOIN 1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_TIVA_ADC
  /* Initialize ADC and register the ADC driver. */




  ret = tm4c_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_adc_setup failed: %d\n", ret);
    }
#endif


#ifdef CONFIG_DEV_GPIO
ret =tm4c_dev_gpio_init();
printf("entering gpio\n");
//sninfo("entering to register gpio\n");

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpio setup failed: %d\n", ret);

      printf("error register gpio\n");
    }
#endif





#ifdef CONFIG_TIVA_CAN
  /* Initialize CAN module and register the CAN driver(s) */

  ret = tm4c_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_can_setup failed %d\n", ret);
    }
#endif

#ifdef HAVE_AT24
  /* Initialize the AT24 driver */

  ret = tm4c_at24_automount(AT24_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tm4c_at24_automount failed: %d\n", ret);
      return ret;
    }
#endif /* HAVE_AT24 */

#ifdef CONFIG_TIVA_TIMER
  /* Initialize the timer driver */

  ret = tiva_timer_configure();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: tiva_timer_configure failed: %d\n", ret);
      return ret;
    }
#endif /* CONFIG_TIVA_TIMER */

#ifdef CONFIG_CAN_MCP2515
  /* Configure and initialize the MCP2515 CAN device */

  ret = tiva_mcp2515initialize("/dev/can0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mcp2515initialize() failed: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return ret;
}
