/****************************************************************************
 * boards/arm/tm4c/raspberrypi-pico/src/tm4c_gpio.c
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

#include <nuttx/config.h>
#include<stdio.h>

#include <sys/types.h>
#include <syslog.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <assert.h>
#include <debug.h>


#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "arm_internal.h"
 #include <arch/tiva/chip.h>

 #include<tiva_gpio.h>

//#include "tm4c_gpio.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/* Output pins. GPIO25 is onboard LED any other outputs could be used.
 */

#define GPIO_OUT1  (GPIO_FUNC_OUTPUT|GPIO_PORTA|GPIO_PIN_2)
//#define GPIO_OUT2  (GPIO_FUNC_OUTPUT | GPIO_PORTF | GPIO_PIN_4)
#define GPIO_IN1 (GPIO_PORTF|GPIO_PIN_4)
#define GPIO_IRQPIN1 (GPIO_FUNC_INTERRUPT|GPIO_FUNC_INPUT|GPIO_PORTF|GPIO_PIN_4|GPIO_INT_MASK |GPIO_INT_RISINGEDGE)

/* Input pins.
 */

#define BOARD_NGPIOOUT 1

#define BOARD_NGPIOIN 0

#define BOARD_NGPIOINT 1





/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tm4cgpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

struct tm4cgpint_dev_s
{
  struct tm4cgpio_dev_s tm4cgpio;
  pin_interrupt_t callback;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value);
static int gpout_write(struct gpio_dev_s *dev, bool value);
#endif

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value);
#endif

#if BOARD_NGPIOINT > 0
static int gpint_read(struct gpio_dev_s *dev, bool *value);
static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback);
static int gpint_enable(struct gpio_dev_s *dev, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1,
 // GPIO_OUT2
  
};

static struct tm4cgpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

#if BOARD_NGPIOIN > 0
static const struct gpio_operations_s gpin_ops =
{
  .go_read   = gpin_read,
  .go_write  = NULL,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpioinputs[BOARD_NGPIOIN] =
{
  GPIO_IN1
};

static struct tm4cgpio_dev_s g_gpin[BOARD_NGPIOIN];
#endif

#if BOARD_NGPIOINT > 0
static const struct gpio_operations_s gpint_ops =
{
  .go_read   = gpint_read,
  .go_write  = NULL,
  .go_attach = gpint_attach,
  .go_enable = gpint_enable,
};

/* This array maps the GPIO pins used as INTERRUPT INPUTS */

static const uint32_t g_gpiointinputs[BOARD_NGPIOINT] =
{
  GPIO_IRQPIN1,
};

static struct tm4cgpint_dev_s g_gpint[BOARD_NGPIOINT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(struct gpio_dev_s *dev, bool *value)
{
  struct tm4cgpio_dev_s *tm4cgpio =
    (struct tm4cgpio_dev_s *)dev;

  DEBUGASSERT(tm4cgpio != NULL && value != NULL);
  DEBUGASSERT(tm4cgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value =  tiva_gpioread(g_gpiooutputs[tm4cgpio->id]);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(struct gpio_dev_s *dev, bool value)
{
  struct tm4cgpio_dev_s *tm4cgpio =
    (struct tm4cgpio_dev_s *)dev;

  DEBUGASSERT(tm4cgpio != NULL);
  DEBUGASSERT(tm4cgpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  tiva_gpiowrite(g_gpiooutputs[tm4cgpio->id], value);
  return OK;
}
#endif

/****************************************************************************
 * Name: gpin_read
 ****************************************************************************/

#if BOARD_NGPIOIN > 0
static int gpin_read(struct gpio_dev_s *dev, bool *value)
{
  struct tm4cgpio_dev_s *tm4cgpio =
    (struct tm4cgpio_dev_s *)dev;

  DEBUGASSERT(tm4cgpio != NULL && value != NULL);
  DEBUGASSERT(tm4cgpio->id < BOARD_NGPIOIN);
  gpioinfo("Reading... pin %d\n", (int)g_gpioinputs[tm4cgpio->id]);

 *value =  tiva_gpioread(g_gpioinputs[tm4cgpio->id]);

 printf("gpio pin value=%d\n",*value);
  return OK;
}
#endif

/****************************************************************************
 * Name: tm4cgpio_interrupt
 ****************************************************************************/

#if BOARD_NGPIOINT > 0
static int tm4cgpio_interrupt(int irq, void *context, void *arg)
{
  struct tm4cgpint_dev_s *tm4cgpint =
    (struct tm4cgpint_dev_s *)arg;

//gpioinfo("Interrupt! id=%x\n",tm4cgpint->tm4cgpio.id);

//printf("call back=%p\n",tm4cgpint->callback);

  DEBUGASSERT(tm4cgpint != NULL && tm4cgpint->callback != NULL);
  printf("Interrupt! callback=%p\n", tm4cgpint->callback);

if(tm4cgpint->callback != NULL)
{
  tm4cgpint->callback(&tm4cgpint->tm4cgpio.gpio,
                       tm4cgpint->tm4cgpio.id);
}

  printf("call back done\n");
  return OK;
}

/****************************************************************************
 * Name: gpint_read
 ****************************************************************************/

static int gpint_read(struct gpio_dev_s *dev, bool *value)
{
  struct tm4cgpint_dev_s *tm4cgpint =
    (struct tm4cgpint_dev_s *)dev;

  DEBUGASSERT(tm4cgpint != NULL && value != NULL);
  DEBUGASSERT(tm4cgpint->tm4cgpio.id < BOARD_NGPIOINT);
  gpioinfo("Reading int pin...\n");

  *value = tiva_gpioread(g_gpiointinputs[tm4cgpint->tm4cgpio.id]);
  return OK;
}

/****************************************************************************
 * Name: gpint_attach
 ****************************************************************************/

static int gpint_attach(struct gpio_dev_s *dev,
                        pin_interrupt_t callback)
{
  struct tm4cgpint_dev_s *tm4cgpint =
    (struct tm4cgpint_dev_s *)dev;
  int irq = g_gpiointinputs[tm4cgpint->tm4cgpio.id];
  int ret;

  gpioinfo("Attaching the callback\n");

  /* Make sure the interrupt is disabled */
   tiva_gpioirqdisable(irq);

  //tm4c_gpio_disable_irq(irq);

   printf("isr vlaue=%p\n",(g_gpint[tm4cgpint->tm4cgpio.id].callback));

  ret = tiva_gpioirqattach(irq,tm4cgpio_interrupt,&g_gpint[tm4cgpint->tm4cgpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }



   gpioinfo("Attach %p\n", callback);
  tm4cgpint->callback = callback;

  return OK;
}

/****************************************************************************
 * Name: gpint_enable
 ****************************************************************************/

static int gpint_enable(struct gpio_dev_s *dev, bool enable)
{

    int ret;

 // printf("enable or disable interrupt\n");
  struct tm4cgpint_dev_s *tm4cgpint =
    (struct tm4cgpint_dev_s *)dev;
  int irq = g_gpiointinputs[tm4cgpint->tm4cgpio.id];

  if (enable)
    {
      if (tm4cgpint->callback != NULL)
        {
          gpioinfo("Enabling the interrupt\n");

          /* Configure the interrupt for rising edge */

          /// tiva_enable_irq(irq);

          tiva_gpioirqenable(irq);

        //  tiva_interrupt(irq);
          //tiva_gpioirqenable(irq);

         // tiva_interrupt(irq);
  
          //tm4c_gpio_enable_irq(irq);
        }
    }
  else
    {
      gpioinfo("Disable the interrupt\n");

printf("remove interrupt attach\n");   
  ret = tiva_gpioirqattach(irq,NULL,&g_gpint[tm4cgpint->tm4cgpio.id]);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: gpint_attach() failed: %d\n", ret);
      return ret;
    }

printf("remove interrupt \n"); 
    


      tiva_gpioirqdisable(irq);

           tiva_gpioirqclear(irq);
 // tiva_gpioirqclear(pinconfig);
     // tm4c_gpio_disable_irq(irq);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_dev_gpio_init
 ****************************************************************************/

int tm4c_dev_gpio_init(void)
{

 // printf("inside gpio init\n");
 // printf("gpio pi  value=%x",GPIO_OUT1);
  int i;
  int pincount = 0;
 

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, g_gpiooutputs[i]);

      /* Configure the pins that will be used as output */


      tiva_configgpio(g_gpiooutputs[i]);

      //tm4c_gpio_init(g_gpiooutputs[i]);
    //  tm4c_gpio_setdir(g_gpiooutputs[i], true);
      //tm4c_gpio_put(g_gpiooutputs[i], false);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOIN > 0
  for (i = 0; i < BOARD_NGPIOIN; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpin[i].gpio.gp_pintype = GPIO_INPUT_PIN;
      g_gpin[i].gpio.gp_ops     = &gpin_ops;
      g_gpin[i].id              = i;
      gpio_pin_register(&g_gpin[i].gpio, g_gpioinputs[i]);
      tiva_configgpio(g_gpioinputs[i]);

      /* Configure the pins that will be used as INPUT */

      //tm4c_gpio_init(g_gpioinputs[i]);

      pincount++;
    }
#endif

  pincount = 0;

#if BOARD_NGPIOINT > 0
  for (i = 0; i < BOARD_NGPIOINT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpint[i].tm4cgpio.gpio.gp_pintype = GPIO_INTERRUPT_PIN;
      g_gpint[i].tm4cgpio.gpio.gp_ops     = &gpint_ops;
      g_gpint[i].tm4cgpio.id              = i;
      gpio_pin_register(&g_gpint[i].tm4cgpio.gpio, g_gpiointinputs[i]);

      tiva_configgpio(g_gpiointinputs[i]);

      /* Configure the pins that will be used as interrupt input */

     // tm4c_gpio_init(g_gpiointinputs[i]);

      /* pull-up = false : pull-down = true */

      //tm4c_gpio_set_pulls(g_gpiointinputs[i], false, true);

      pincount++;
    }
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
