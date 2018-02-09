/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @brief          : Header for usbd_cdc_if file.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef USBD_USE_CDC

#include "usbd_cdc.h"

#ifdef __cplusplus

#include "Stream.h"

#if CDC_DATA_HS_OUT_PACKET_SIZE > CDC_DATA_FS_OUT_PACKET_SIZE
#define CDC_MIN_BUFFER_SIZE (CDC_DATA_HS_OUT_PACKET_SIZE*2)
#else
#define CDC_MIN_BUFFER_SIZE (CDC_DATA_FS_OUT_PACKET_SIZE*2)
#endif

#ifndef CDC_SERIAL_BUFFER_SIZE
#define CDC_SERIAL_BUFFER_SIZE  CDC_MIN_BUFFER_SIZE
#else
#if CDC_SERIAL_BUFFER_SIZE < CDC_MIN_BUFFER_SIZE
#undef CDC_SERIAL_BUFFER_SIZE
#define CDC_SERIAL_BUFFER_SIZE  CDC_MIN_BUFFER_SIZE
#endif
#endif

class SerialUSBClass : public Stream
{
//private:
public:

  enum { BUFFER_SIZE = 64, BUFFER_COUNT = CDC_SERIAL_BUFFER_SIZE/BUFFER_SIZE };
  struct pool_buffer
  {
    uint8_t buffer[BUFFER_SIZE];
    volatile uint32_t index;
    volatile uint32_t count;
  };

  pool_buffer _rx_buffer[BUFFER_COUNT];
  volatile uint32_t _head;
  volatile uint32_t _tail;

  USBD_CDC_LineCodingTypeDef _usbLineInfo;

  uint8_t _lineState;
  int _breakValue;

  friend int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
  friend int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t Len);

  void accept();

public:
  SerialUSBClass();
  void begin(uint32_t baud_count);
  void begin(uint32_t baud_count, uint8_t config);
  void end(void);

  virtual int available(void);
  virtual int availableForWrite(void);
  virtual int peek(void);
  virtual int read(void);
  virtual void flush(void);
  virtual size_t write(uint8_t);
  virtual size_t write(const uint8_t *buffer, size_t size);
  using Print::write; // pull in write(str) from Print
  operator bool();

  // This method allows processing "SEND_BREAK" requests sent by
  // the USB host. Those requests indicate that the host wants to
  // send a BREAK signal and are accompanied by a single uint16_t
  // value, specifying the duration of the break. The value 0
  // means to end any current break, while the value 0xffff means
  // to start an indefinite break.
  // readBreak() will return the value of the most recent break
  // request, but will return it at most once, returning -1 when
  // readBreak() is called again (until another break request is
  // received, which is again returned once).
  // This also mean that if two break requests are received
  // without readBreak() being called in between, the value of the
  // first request is lost.
  // Note that the value returned is a long, so it can return
  // 0-0xffff as well as -1.
  int32_t readBreak();

  // These return the settings specified by the USB host for the
  // serial port. These aren't really used, but are offered here
  // in case a sketch wants to act on these settings.
  uint32_t baud();
  uint8_t stopbits();
  uint8_t paritytype();
  uint8_t numbits();
  bool dtr();
  bool rts();
  enum {
    ONE_STOP_BIT = 0,
    ONE_AND_HALF_STOP_BIT = 1,
    TWO_STOP_BITS = 2,
  };
  enum {
    NO_PARITY = 0,
    ODD_PARITY = 1,
    EVEN_PARITY = 2,
    MARK_PARITY = 3,
    SPACE_PARITY = 4,
  };
};
extern SerialUSBClass SerialUSB;

extern "C" {
#endif // __cplusplus

extern USBD_CDC_ItfTypeDef  USBD_Interface_fops_FS;
  
#ifdef __cplusplus
}
#endif

#endif // USBD_USE_CDC

#endif /* __USBD_CDC_IF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
