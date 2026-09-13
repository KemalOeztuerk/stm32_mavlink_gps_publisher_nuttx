/****************************************************************************
 * boards/arm/stm32f4/mavlink-f446/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __BOARDS_ARM_STM32F4_MAVLINK_F446_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F4_MAVLINK_F446_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz, the same crystal the
 *       STM32F105RB variant of this design carries.
 * LSE - not fitted
 *
 * The PLL is fed from HSE rather than the internal RC: a 1Mbit DroneCAN
 * link has no room for the HSI's +/-1% trim tolerance.
 *
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000   (STM32_PLLCFG_PLLP)
 *   HCLK(Hz)                      : 180000000   (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1
 *   APB1 Prescaler                : 4            => 45MHz (APB1 maximum)
 *   APB2 Prescaler                : 2            => 90MHz (APB2 maximum)
 *   HSE Frequency(Hz)             : 8000000     (STM32_BOARD_XTAL)
 *   PLLM                          : 4            => 2MHz VCO input
 *   PLLN                          : 180          => 360MHz VCO output
 *   PLLP                          : 2            => 180MHz SYSCLK
 *   PLLQ                          : 8            (unused: no USB/SDIO here)
 *   Flash Latency(WS)             : 5
 *
 * 180MHz needs the F446's over-drive mode; stm32f40xxx_rcc.c switches it on
 * by itself for SYSCLK > 168MHz on parts that have it, so nothing here has
 * to ask for it.
 *
 * If you respin this board with a different crystal, PLLM is the only value
 * that changes -- keep the VCO input at 2MHz: PLLM = XTAL / 2MHz (so 12MHz
 * => 6, 16MHz => 8, 25MHz => 12.5, which is why 25MHz crystals instead use
 * PLLM=25 with PLLN=360).
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 *   PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *           = (8,000,000 / 4) * 180
 *           = 360 MHz
 *   SYSCLK  = PLL_VCO / PLLP
 *           = 360,000,000 / 2
 *           = 180 MHz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(180)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)

#define STM32_SYSCLK_FREQUENCY  180000000ul

/* AHB clock (HCLK) is SYSCLK (180MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (45MHz, the APB1 maximum).
 *
 * This is also the CAN1 clock: 45MHz / 1Mbit = 45 time quanta, which the
 * nsh defconfig splits as brp=3 x (1 + TSEG1(12) + TSEG2(2)) = 45 for an
 * exact 1Mbit with the sample point at 86.7%.
 */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (90MHz, the APB2 maximum).
 *
 * USART1 (the MAVLink link) is on APB2: 90MHz / (16 * 57600) = 97.66, which
 * the fractional divider reaches within 0.02%.
 */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Alternate function pin selections ***************************************/

/* Identical to the STM32F105RB board's pinout -- both parts put USART1 and
 * CAN1 on the same package pins, so the same PCB footprint works with
 * either chip fitted.
 */

/* USART1: MAVLink link to the companion computer/autopilot.
 * PA9=TX, PA10=RX (AF7).
 */

#define GPIO_USART1_TX  (GPIO_USART1_TX_1 | GPIO_SPEED_50MHz)   /* PA9  */
#define GPIO_USART1_RX  (GPIO_USART1_RX_1 | GPIO_SPEED_50MHz)   /* PA10 */

/* CAN1: Here4 DroneCAN GNSS/compass/IMU/baro. PA11=RX, PA12=TX (AF9). */

#define GPIO_CAN1_RX    (GPIO_CAN1_RX_1 | GPIO_SPEED_50MHz)     /* PA11 */
#define GPIO_CAN1_TX    (GPIO_CAN1_TX_1 | GPIO_SPEED_50MHz)     /* PA12 */

#endif /* __BOARDS_ARM_STM32F4_MAVLINK_F446_INCLUDE_BOARD_H */
