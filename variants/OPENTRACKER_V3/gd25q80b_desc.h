/**
  ******************************************************************************
  * @file    mx25r6435f.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-February-2017
  * @brief   This file contains all the description of the GD25Q80B QSPI memory.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GD25Q80B_H
#define __GD25Q80B_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 
  
/** @addtogroup quadspiflash
  * @{
  */

/** @defgroup GD25Q80B_Exported_Types
  * @{
  */
   
/**
  * @}
  */ 

/** @defgroup GD25Q80B_Exported_Constants
  * @{
  */
   
/** 
  * @brief  GD25Q80B Configuration  
  */  
#define GD25Q80B_FLASH_SIZE                0x100000  /* 8 MBits => 1MBytes */
#define GD25Q80B_BLOCK_SIZE                0x10000   /* 16 blocks of 64KBytes */
#define GD25Q80B_SUBBLOCK_SIZE             0x8000    /* 32 blocks of 32KBytes */
#define GD25Q80B_SECTOR_SIZE               0x1000    /* 256 sectors of 4kBytes */
#define GD25Q80B_PAGE_SIZE                 0x100     /* 4096 pages of 256 bytes */

#define GD25Q80B_DUMMY_CYCLES_READ         8
#define GD25Q80B_DUMMY_CYCLES_READ_DUAL    4
#define GD25Q80B_DUMMY_CYCLES_READ_QUAD    4
#define GD25Q80B_DUMMY_CYCLES_2READ        2
#define GD25Q80B_DUMMY_CYCLES_4READ        4

#define GD25Q80B_ALT_BYTES_PE_MODE         0xA5
#define GD25Q80B_ALT_BYTES_NO_PE_MODE      0x00

#define GD25Q80B_CHIP_ERASE_MAX_TIME       20000
#define GD25Q80B_BLOCK_ERASE_MAX_TIME      1200
#define GD25Q80B_SUBBLOCK_ERASE_MAX_TIME   1000
#define GD25Q80B_SECTOR_ERASE_MAX_TIME     300

/** 
  * @brief  GD25Q80B Commands  
  */  
/* Read Operations */
#define READ_CMD                             0x03
#define FAST_READ_CMD                        0x0B
#define DUAL_OUT_READ_CMD                    0x3B
#define DUAL_INOUT_READ_CMD                  0xBB
#define QUAD_OUT_READ_CMD                    0x6B
#define QUAD_INOUT_READ_CMD                  0xEB

/* Program Operations */
#define PAGE_PROG_CMD                        0x02
#define QUAD_PAGE_PROG_CMD                   0x32

/* Erase Operations */
#define SECTOR_ERASE_CMD                     0x20
#define SUBBLOCK_ERASE_CMD                   0x52
#define BLOCK_ERASE_CMD                      0xD8
#define CHIP_ERASE_CMD                       0x60
#define CHIP_ERASE_CMD_2                     0xC7

#define PROG_ERASE_RESUME_CMD                0x7A
#define PROG_ERASE_SUSPEND_CMD               0x75

/* Identification Operations */
#define READ_ID_CMD                          0x9F
#define READ_ELECTRONIC_ID_CMD               0xAB
#define READ_ELEC_MANUFACTURER_DEVICE_ID_CMD 0x90

/* Write Operations */
#define WRITE_ENABLE_CMD                     0x06
#define WRITE_DISABLE_CMD                    0x04

/* Register Operations */
#define READ_STATUS_REG_CMD                  0x05
#define READ_STATUS1_REG_CMD                 0x35
#define WRITE_STATUS_REG_CMD                 0x01

#define READ_SEC_REG_CMD                     0x48
#define WRITE_SEC_REG_CMD                    0x42
#define ERASE_SEC_REG_CMD                    0x44

/* Power Down Operations */
#define DEEP_POWER_DOWN_CMD                  0xB9
#define RELEASE_POWER_DOWN_CMD               0xAB

/* High Performance */
#define SET_HIGH_PERF_CMD                    0xA3
#define RELEASE_HIGH_PERF_CMD                0xAB

/* One-Time Programmable Operations */
//~ #define ENTER_SECURED_OTP_CMD                0xB1
//~ #define EXIT_SECURED_OTP_CMD                 0xC1

/* Reset Operations */
#define RELEASE_READ_ENHANCED_CMD            0xFF

/** 
  * @brief  GD25Q80B Registers  
  */ 
/* Status Register */
#define GD25Q80B_SR_WIP                    ((uint8_t)0x01)    /*!< Write in progress */
#define GD25Q80B_SR_WEL                    ((uint8_t)0x02)    /*!< Write enable latch */
#define GD25Q80B_SR_BP                     ((uint8_t)0x7C)    /*!< Block protect */
#define GD25Q80B_SR_SRWD                   ((uint8_t)0x80)    /*!< Status register write disable */
#define GD25Q80B_SR1_QE                    ((uint8_t)0x02)    /*!< Quad enable */
#define GD25Q80B_SR1_SUS                   ((uint8_t)0x80)    /*!< Suspend status */

/**
  * @}
  */
  
/** @defgroup GD25Q80B_Exported_Functions
  * @{
  */ 
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */
  
#ifdef __cplusplus
}
#endif

#endif /* __GD25Q80B_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
