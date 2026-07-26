/****************************************************************************
 * boards/arm/stm32f1/mavlink-f105/src/stm32_spi.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"

#include "mavlink_f105.h"

#if defined(CONFIG_STM32_SPI1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure the SPI1 chip select GPIO used by the MPU9250 on
 *   the mavlink-f105 board.
 *
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  stm32_configgpio(GPIO_MPU9250_CS);
}

/****************************************************************************
 * Name: stm32_spi1select and stm32_spi1status
 *
 * Description:
 *   The external functions stm32_spi1select and stm32_spi1status must be
 *   provided by board-specific logic.  They are implementations of the
 *   select and status methods of the SPI interface defined by struct
 *   spi_ops_s (see include/nuttx/spi/spi.h). All other methods (including
 *   stm32_spibus_initialize()) are provided by common STM32 logic.
 *
 ****************************************************************************/

void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %d CS: %s\n",
          (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_IMU(0))
    {
      /* Set the GPIO low to select and high to de-select */

      stm32_gpiowrite(GPIO_MPU9250_CS, !selected);
    }
}

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: board_mpu9250_spibus
 *
 * Description:
 *   Returns the SPI1 bus handle the MPU9250 IMU is wired to, initializing
 *   it on first call. See the prototype comment in include/board.h for why
 *   this indirection exists.
 *
 ****************************************************************************/

struct spi_dev_s *board_mpu9250_spibus(void)
{
  static struct spi_dev_s *spi;

  if (spi == NULL)
    {
      spi = stm32_spibus_initialize(1);
    }

  return spi;
}

#endif /* CONFIG_STM32_SPI1 */
