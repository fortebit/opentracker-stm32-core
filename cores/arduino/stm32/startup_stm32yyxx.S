#include "stm32_def_build.h"

#include CMSIS_STARTUP_FILE

/* Bootloader entry check done early at reset (then jump into original entry point) */

    .section	.text.Boot_Reset_Handler
	.type	Boot_Reset_Handler, %function
	.thumb_set Reset_Handler,Boot_Reset_Handler
Boot_Reset_Handler:
  ldr   sp, =_estack    /* Atollic update: set stack pointer */
  bl    checkBoot
/* Copy the data segment initializers from flash to SRAM */
  movs	r1, #0
  b	LoopCopyDataInit    /* Jump to original Reset_Handler */
    
.size	Boot_Reset_Handler, .-Boot_Reset_Handler
