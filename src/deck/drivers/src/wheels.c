/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2015 BitCraze AB
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
 * wheels.c - Deck driver for the picobug wheel deck
 * based off of buzzdeck.c
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "timers.h"

#include "deck.h"
#include "param.h"


//THIS IS WHERE THE MAGIC HAPPENS right down there
#include "wheel_driver.h"  //equivalent to piezo

#define WHEEL_FWD HIGH
#define WHEEL_BWD LOW

// IN1 is mapped to deck expansion pin IO1
// IN2 is mapped to deck expansion pin IO2
#define WHEEL_IN_1  DECK_GPIO_IO1
#define WHEEL_IN_2  DECK_GPIO_IO2

/* Enumeration for setting different states */
typedef enum {
	stop = 0,
    forward = 1,
    left = 2,
    right = 3,
    backward = 4
} Wheels;
static Wheels wheels;

//params for controlling the pwm of wheels 1 and 2
static uint8_t pwm1;
static uint8_t pwm2;


static void wheelDeckPWM(uint8_t power1, uint8_t power2)
{
	wheelDriverSetRatio(power1,power2);
	wheelDriverSetFreq(10000);
}

static void wheelDeckOff()
{
  wheelDriverSetRatio(0,0);
}

/* Timer loop and handle */
static xTimerHandle timer;
static void wheelsTimer(xTimerHandle timer)
{
  switch (wheels) {
     case stop:
      wheelDeckOff();
      break;
    case forward:
      wheelDeckPWM(pwm1,pwm2);
      digitalWrite(WHEEL_IN_1, WHEEL_FWD);
      digitalWrite(WHEEL_IN_2, WHEEL_FWD);
      break;
    case left:
      wheelDeckPWM(pwm1,pwm2);
      digitalWrite(WHEEL_IN_1, WHEEL_FWD);
      digitalWrite(WHEEL_IN_2, WHEEL_BWD);
      break;
    case right:
      wheelDeckPWM(pwm1,pwm2);
      digitalWrite(WHEEL_IN_1, WHEEL_BWD);
      digitalWrite(WHEEL_IN_2, WHEEL_FWD);
      break;
    case backward:
      wheelDeckPWM(pwm1,pwm2);
      digitalWrite(WHEEL_IN_1, WHEEL_BWD);
      digitalWrite(WHEEL_IN_2, WHEEL_BWD);
      break;
    default:
      /* Keep all LEDs off */
      break;
  }
}

static void wheelDeckInit(DeckInfo *info)
{
  wheelDriverInit();
  pinMode(WHEEL_IN_1, OUTPUT);
  pinMode(WHEEL_IN_2, OUTPUT);
  digitalWrite(WHEEL_IN_1, WHEEL_FWD);
  digitalWrite(WHEEL_IN_2, WHEEL_FWD);

  timer = xTimerCreate( "wheelsTimer", M2T(10),
                         pdTRUE, NULL, wheelsTimer );
  xTimerStart(timer, 100);
}

PARAM_GROUP_START(wheels)
PARAM_ADD(PARAM_UINT8, state, &wheels)
PARAM_ADD(PARAM_UINT8, pwm_1, &pwm1)
PARAM_ADD(PARAM_UINT8, pwm_2, &pwm2)
PARAM_GROUP_STOP(wheels)

static const DeckDriver wheel_deck = {
  .vid = 0,
  .pid = 0,
  .name = "wheelDeck",

  //TIMER5 is used by the pwm in wheel_driver.c
  //TX2 and RX2 are the PWM signals
  //PB8 is IO1, PB5 is IO2
  .usedPeriph = DECK_USING_TIMER5,
  .usedGpio = DECK_USING_TX2 | DECK_USING_RX2 | DECK_USING_PB5 | DECK_USING_PB8,

  .init = wheelDeckInit,
};

DECK_DRIVER(wheel_deck);
