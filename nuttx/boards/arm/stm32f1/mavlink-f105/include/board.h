/****************************************************************************
 * boards/arm/stm32f1/mavlink-f105/include/board.h
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

#ifndef __BOARDS_ARM_STM32F1_MAVLINK_F105_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F1_MAVLINK_F105_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <nuttx/spi/spi.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 8MHz
 * LSE - not fitted
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* This is a Connectivity Line part, so the common clock-configuration
 * logic (stm32_stdclockconfig() in stm32f10xxx_rcc.c) always routes
 * PREDIV1 through PLL2 -- there is no direct HSE->PREDIV1 path for this
 * chip family, even though the original firmware's SystemClock_Config()
 * fed PREDIV1 from HSE directly. Reaching the same 72MHz SYSCLK/HCLK via
 * PLL2 is electrically equivalent for every peripheral on this board:
 *
 *   HSE(8MHz) / PREDIV2(/2) = 4MHz -> PLL2 x8 = 32MHz
 *   PLL2CLK(32MHz) / PREDIV1(/4) = 8MHz -> main PLL x9 = 72MHz
 */

#define STM32_PLL_PREDIV2       RCC_CFGR2_PREDIV2d2   /* 8MHz / 2 => 4MHz */
#define STM32_PLL_PLL2MUL       RCC_CFGR2_PLL2MULx8    /* 4MHz * 8 => 32MHz */
#define STM32_PLL_PREDIV1       RCC_CFGR2_PREDIV1d4    /* 32MHz / 4 => 8MHz */
#define STM32_PLL_PLLMUL        RCC_CFGR_PLLMUL_CLKx9  /* 8MHz * 9 => 72MHz */
#define STM32_PLL_FREQUENCY     (72000000ul)

/* SYSCLK and HCLK are the PLL frequency */

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 2-7, 12-14 */

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_TIM1_CLKIN        (STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_TIM2_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN        (2 * STM32_PCLK1_FREQUENCY)

/* Alternate function pin selections ***************************************/

/* USART1: MAVLink link to the companion computer/autopilot.
 * PA9=TX, PA10=RX (no remap).
 */

#define GPIO_USART1_TX  GPIO_ADJUST_MODE(GPIO_USART1_TX_0, GPIO_MODE_50MHz)
#define GPIO_USART1_RX  GPIO_USART1_RX_0

/* USART2: GPS NMEA link. PA2=TX, PA3=RX (no remap). */

#define GPIO_USART2_TX  GPIO_ADJUST_MODE(GPIO_USART2_TX_0, GPIO_MODE_50MHz)
#define GPIO_USART2_RX  GPIO_USART2_RX_0

/* SPI1: MPU9250 IMU. PA5=SCK, PA6=MISO, PA7=MOSI (no remap). PA4 is the
 * chip select, driven as a plain GPIO by board logic (software NSS
 * management) -- see stm32_spi1select() in src/stm32_spi.c -- so the
 * hardware NSS pin alias is intentionally not defined here.
 */

#define GPIO_SPI1_SCK   GPIO_ADJUST_MODE(GPIO_SPI1_SCK_0, GPIO_MODE_50MHz)
#define GPIO_SPI1_MISO  GPIO_SPI1_MISO_0
#define GPIO_SPI1_MOSI  GPIO_ADJUST_MODE(GPIO_SPI1_MOSI_0, GPIO_MODE_50MHz)

/* CAN1: Here4 DroneCAN GNSS/compass. PA11=RX, PA12=TX (no remap). */

#define GPIO_CAN1_RX    GPIO_CAN1_RX_0
#define GPIO_CAN1_TX    GPIO_ADJUST_MODE(GPIO_CAN1_TX_0, GPIO_MODE_50MHz)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: board_mpu9250_spibus
 *
 * Description:
 *   Returns the SPI1 bus handle the MPU9250 IMU is wired to, initializing
 *   it on first call. This is the only arch-level SPI entry point exposed
 *   to application code; it exists so that apps/examples/
 *   mavlink_gps_publisher (which implements its own MPU9250 driver in
 *   application space rather than as a NuttX character driver) can reach
 *   the bus without linking against arch-private headers.
 *
 ****************************************************************************/

struct spi_dev_s *board_mpu9250_spibus(void);

#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_STM32F1_MAVLINK_F105_INCLUDE_BOARD_H */
