/*
  Copyright (c) 2012 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "stm32_def.h"
#include "Reset.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RESET_TO_BOOTLOADER_MAGIC_CODE 0xDEADBEEF

uint32_t reset_to_bootloader_magic __attribute__ ((section (".noinit")));

void checkBoot()
{
  if (reset_to_bootloader_magic == RESET_TO_BOOTLOADER_MAGIC_CODE) {
    reset_to_bootloader_magic = 0;
#if defined(MCU_SERIES_F7)
    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
    // MSP function, so we write it out explicitly here.
    //__set_MSP(*((uint32_t*) 0x1FF00000));
    __ASM volatile ("movw r3, #0x0000\nmovt r3, #0x1FF0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x1FF00004))();
#else
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    // arm-none-eabi-gcc 4.9.0 does not correctly inline this
    // MSP function, so we write it out explicitly here.
    //__set_MSP(*((uint32_t*) 0x00000000));
    __ASM volatile ("movs r3, #0\nldr r3, [r3, #0]\nMSR msp, r3\n" : : : "r3", "sp");

    ((void (*)(void)) *((uint32_t*) 0x00000004))();
#endif
  }
}

// Activate the bootloader without BOOT* pins.
void banzai() {
  reset_to_bootloader_magic = RESET_TO_BOOTLOADER_MAGIC_CODE;
  NVIC_SystemReset();
}

static int ticks = -1;

void initiateReset(int _ticks) {
  ticks = _ticks;
}

void cancelReset() {
  ticks = -1;
}

void tickReset() {
  if (ticks == -1)
    return;
  ticks--;
  if (ticks == 0)
    banzai();
}

#ifdef __cplusplus
}
#endif
