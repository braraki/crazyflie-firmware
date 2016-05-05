/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * wheel_driver.c - Wheel driver
 *
 * This code mainly interfacing the PWM peripheral lib of ST.
 *
 * This code was based off of piezo.c, which was in turn based off
 * of motors.c
 */

#include <stdbool.h>

/* ST includes */
#include "stm32fxxx.h"

#include "wheel_driver.h"

//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

// TIMER5, CH3 and CH4
// HW defines
#define WHEEL_TIM_PERIF       RCC_APB1Periph_TIM5
#define WHEEL_TIM             TIM5
#define WHEEL_TIM_DBG         DBGMCU_TIM2_STOP
#define WHEEL_TIM_SETCOMPARE  TIM_SetCompare2
#define WHEEL_TIM_GETCAPTURE  TIM_GetCapture2

#define WHEEL_GPIO_1_PERIF         RCC_AHB1Periph_GPIOA
#define WHEEL_GPIO_1_PORT          GPIOA
#define WHEEL_GPIO_1_PIN           GPIO_Pin_2 // TIM5_CH3
#define WHEEL_GPIO_AF_1_PIN        GPIO_PinSource2
#define WHEEL_GPIO_AF_1            GPIO_AF_TIM5

#define WHEEL_GPIO_2_PERIF         RCC_AHB1Periph_GPIOA
#define WHEEL_GPIO_2_PORT          GPIOA
#define WHEEL_GPIO_2_PIN           GPIO_Pin_3 // TIM5_CH4
#define WHEEL_GPIO_AF_2_PIN        GPIO_PinSource3
#define WHEEL_GPIO_AF_2            GPIO_AF_TIM5

#define WHEEL_PWM_BITS      (8)
#define WHEEL_PWM_PERIOD    ((1<<WHEEL_PWM_BITS) - 1)
#define WHEEL_PWM_PRESCALE  (0)

/* This should be calculated.. */
#define WHEEL_BASE_FREQ (329500)

static bool isInit = false;

/* Public functions */

void wheelDriverInit()
{
  if (isInit)
    return;

  //Init structures
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;

  //Clock the gpio and the timers
  RCC_AHB1PeriphClockCmd(WHEEL_GPIO_1_PERIF | WHEEL_GPIO_2_PERIF, ENABLE);
  RCC_APB1PeriphClockCmd(WHEEL_TIM_PERIF, ENABLE);

  // Configure the GPIO for the timer output
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

  GPIO_InitStructure.GPIO_Pin = WHEEL_GPIO_1_PIN;
  GPIO_Init(WHEEL_GPIO_1_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = WHEEL_GPIO_2_PIN;
  GPIO_Init(WHEEL_GPIO_2_PORT, &GPIO_InitStructure);

  //Map timers to alternate functions
  GPIO_PinAFConfig(WHEEL_GPIO_1_PORT, WHEEL_GPIO_AF_1_PIN, WHEEL_GPIO_AF_1);
  GPIO_PinAFConfig(WHEEL_GPIO_2_PORT, WHEEL_GPIO_AF_2_PIN, WHEEL_GPIO_AF_2);

  //Timer configuration
  TIM_TimeBaseStructure.TIM_Period = WHEEL_PWM_PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = WHEEL_PWM_PRESCALE;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(WHEEL_TIM, &TIM_TimeBaseStructure);

  // PWM channels configuration
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  // Configure OC3
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(WHEEL_TIM, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(WHEEL_TIM, TIM_OCPreload_Enable);

  // Configure OC4 inverted
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(WHEEL_TIM, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(WHEEL_TIM, TIM_OCPreload_Enable);

  //Enable the timer PWM outputs
  TIM_CtrlPWMOutputs(WHEEL_TIM, ENABLE);
  TIM_SetCompare3(WHEEL_TIM, 0x00);
  TIM_SetCompare4(WHEEL_TIM, 0x00);

  //Enable the timer
  TIM_Cmd(WHEEL_TIM, ENABLE);

  isInit = true;
}

bool wheelDriverTest(void)
{
  return isInit;
}

//set the PWM duty cycle from 0 to 255
void wheelDriverSetRatio(uint8_t ratio1, uint8_t ratio2)
{
  TIM_SetCompare3(WHEEL_TIM, ratio1);
  TIM_SetCompare4(WHEEL_TIM, ratio2);
}

//set PWM frequency
void wheelDriverSetFreq(uint16_t freq)
{
  TIM_PrescalerConfig(WHEEL_TIM, (WHEEL_BASE_FREQ/freq), TIM_PSCReloadMode_Update);
}
