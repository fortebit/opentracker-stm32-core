/**
  ******************************************************************************
  * @file           : usbd_cdc_if.cpp
  * @brief          :
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
#ifdef USBD_USE_CDC

#include "Arduino.h"
#include "Reset.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int8_t CDC_Init_FS     (void);
int8_t CDC_DeInit_FS   (void);
int8_t CDC_Control_FS  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
int8_t CDC_Receive_FS  (uint8_t* pbuf, uint32_t Len);
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* For information purpose only since RTS is not always handled by the terminal application */
#define CDC_LINESTATE_DTR   0x01 // Data Terminal Ready
#define CDC_LINESTATE_RTS   0x02 // Ready to Send

#define CDC_LINESTATE_READY   (CDC_LINESTATE_RTS | CDC_LINESTATE_DTR)

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CDC_Init_FS
  *         Initializes the CDC media low layer over the FS USB IP
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Init_FS(void)
{
  CDC_Receive_FS(NULL, 0);
  return (USBD_OK);
}

/**
  * @brief  CDC_DeInit_FS
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_DeInit_FS(void)
{
  SerialUSB._lineState = 0; // force closed
  return (USBD_OK);
}

/**
  * @brief  CDC_Control_FS
  *         Manage the CDC class requests
  * @param  cmd: Command code            
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
  switch (cmd)
  {
  case CDC_SEND_ENCAPSULATED_COMMAND:
    /* Add your code here */
    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:
    /* Add your code here */
    break;

  case CDC_SET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_GET_COMM_FEATURE:
    /* Add your code here */
    break;

  case CDC_CLEAR_COMM_FEATURE:
    /* Add your code here */
    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */ 
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
  case CDC_SET_LINE_CODING:
    SerialUSB._usbLineInfo.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
    SerialUSB._usbLineInfo.format     = pbuf[4];
    SerialUSB._usbLineInfo.paritytype = pbuf[5];
    SerialUSB._usbLineInfo.datatype   = pbuf[6];
    break;

  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(SerialUSB._usbLineInfo.bitrate);
    pbuf[1] = (uint8_t)(SerialUSB._usbLineInfo.bitrate >> 8);
    pbuf[2] = (uint8_t)(SerialUSB._usbLineInfo.bitrate >> 16);
    pbuf[3] = (uint8_t)(SerialUSB._usbLineInfo.bitrate >> 24);
    pbuf[4] = SerialUSB._usbLineInfo.format;
    pbuf[5] = SerialUSB._usbLineInfo.paritytype;
    pbuf[6] = SerialUSB._usbLineInfo.datatype;
    break;

  case CDC_SET_CONTROL_LINE_STATE:
    SerialUSB._lineState = pbuf[2];
    // auto-reset into the bootloader is triggered when the port, already
    // open at 1200 bps, is closed.
    if (SerialUSB.baud() == 1200)
    {
      // We check DTR state to determine if host port is open (bit 0 of lineState).
      if (SerialUSB.dtr() == 0)
        initiateReset(250);
      else
        cancelReset();
    }
    break;

  case CDC_SEND_BREAK:
    SerialUSB._breakValue = (uint16_t)(pbuf[0] | (pbuf[1] << 8)) & 0xFFFF;
    break;    
    
  default:
    break;
  }
  
  return (USBD_OK);
}

/**
  * @brief  CDC_Receive_FS
  *         Data received over USB OUT endpoint are sent over CDC interface 
  *         through this function.
  *           
  *         @note
  *         This function will block any OUT packet reception on USB endpoint 
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result 
  *         in receiving more data while previous ones are still not sent.
  *                 
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t Len)
{
  //printf("h=%u t=%u l=%u . ", SerialUSB._rx_buffer.head, SerialUSB._rx_buffer.tail, Len);

  if (Len > 0)
  {
    SerialUSB._rx_buffer[SerialUSB._head].index = 0;
    SerialUSB._rx_buffer[SerialUSB._head].count = Len;
    SerialUSB._head = (unsigned int)(SerialUSB._head + 1) % SerialUSBClass::BUFFER_COUNT;
  }
  //printf("h=%u . ", SerialUSB._rx_buffer.head);

  SerialUSB.accept();

  return (USBD_OK);
}

/**
  * @brief  CDC_Transmit_FS
  *         Data send over USB IN endpoint are sent over CDC interface 
  *         through this function.           
  *         @note
  *         
  *                 
  * @param  Buf: Buffer of data to be send
  * @param  Len: Number of data to be send (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;

  if (Len > 0) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0){
      return USBD_BUSY;
    }
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
  return result;
}

SerialUSBClass::SerialUSBClass()
{
  _head = _tail = 0;

  _usbLineInfo.bitrate = 115200;
  _usbLineInfo.format = ONE_STOP_BIT;
  _usbLineInfo.paritytype = NO_PARITY;
  _usbLineInfo.datatype = 8;

  _lineState = 0;
  _breakValue = -1;
}

void SerialUSBClass::begin(uint32_t baud_count)
{
  // suppress "unused parameter" warning
  (void)baud_count;
}

void SerialUSBClass::begin(uint32_t baud_count, uint8_t config)
{
  // suppress "unused parameter" warning
  (void)baud_count;
  (void)config;
}

void SerialUSBClass::end(void)
{
}

int SerialUSBClass::available(void)
{
  unsigned int r = (BUFFER_COUNT + _head - _tail) % BUFFER_COUNT;
  if (r > 0)
    return _rx_buffer[_tail].count - _rx_buffer[_tail].index;
  return 0;
}

int SerialUSBClass::availableForWrite(void)
{
  // buffer is flushed on every write
  return (hUsbDeviceFS.dev_speed == USBD_SPEED_HIGH) ? CDC_DATA_HS_OUT_PACKET_SIZE : CDC_DATA_FS_OUT_PACKET_SIZE;
}

int SerialUSBClass::peek(void)
{
  if (_head == _tail)
  {
    return -1;
  }
  else
  {
    return _rx_buffer[_tail].buffer[_rx_buffer[_tail].index];
  }
}

void SerialUSBClass::accept()
{
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || _lineState == 0)
    return;

  bool ena = (NVIC_GetEnableIRQ(USBD_IRQ_NUM) != 0);
  NVIC_DisableIRQ(USBD_IRQ_NUM);

  unsigned int left = (unsigned int)(BUFFER_COUNT + _tail - _head) % BUFFER_COUNT;
  if (left > 1)
  {
    // setup next transfer
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, _rx_buffer[_head].buffer, BUFFER_SIZE);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }

  if (ena)
    NVIC_EnableIRQ(USBD_IRQ_NUM);
}

int SerialUSBClass::read(void)
{
  if (_head == _tail)
  {
    return -1;
  }
  else
  {
    unsigned char c = _rx_buffer[_tail].buffer[_rx_buffer[_tail].index++];
    if (_rx_buffer[_tail].index >= _rx_buffer[_tail].count)
    {
      _tail = (unsigned int)(_tail + 1) % BUFFER_COUNT;
      accept();
    }
    return c;
  }
}

void SerialUSBClass::flush(void)
{
  //TODO
}

size_t SerialUSBClass::write(const uint8_t *buffer, size_t size)
{
  if (size <= 0)
    return 0;

  /* only try to send bytes if the high-level CDC connection itself
   is open (not just the pipe) - the OS should set lineState when the port
   is opened and clear lineState when the port is closed.
   bytes sent before the user opens the connection or after
   the connection is closed are lost - just like with a UART. */

  // TODO - ZE - check behavior on different OSes and test what happens if an
  // open connection isn't broken cleanly (cable is yanked out, host dies
  // or locks up, or host virtual serial port hangs)
  while (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && _lineState > 0)
  {
    if (CDC_Transmit_FS((uint8_t*)buffer, size) == USBD_OK)
      return size;
    __WFI();
  }

  setWriteError();
  return 0;
}

size_t SerialUSBClass::write(uint8_t c)
{
  return write(&c, 1);
}

// This operator is a convenient way for a sketch to check whether the
// port has actually been configured and opened by the host (as opposed
// to just being connected to the host).  It can be used, for example, in
// setup() before printing to ensure that an application on the host is
// actually ready to receive and display the data.
// We add a short delay before returning to fix a bug observed by Federico
// where the port is configured (lineState != 0) but not quite opened.
SerialUSBClass::operator bool()
{
  // this is here to avoid spurious opening after upload
  if (millis() < 500)
    return false;

  bool result = false;

  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && _lineState > 0)
  {
    result = true;
    delay(10);
  }

  return result;
}

int32_t SerialUSBClass::readBreak()
{
  bool ena = (NVIC_GetEnableIRQ(USBD_IRQ_NUM) != 0);
  NVIC_DisableIRQ(USBD_IRQ_NUM);

  int ret = _breakValue;

  _breakValue = -1;

  if (ena)
    NVIC_EnableIRQ(USBD_IRQ_NUM);

  return ret;
}

unsigned long SerialUSBClass::baud()
{
  return _usbLineInfo.bitrate;
}

uint8_t SerialUSBClass::stopbits()
{
  return _usbLineInfo.format;
}

uint8_t SerialUSBClass::paritytype()
{
  return _usbLineInfo.paritytype;
}

uint8_t SerialUSBClass::numbits()
{
  return _usbLineInfo.datatype;
}

bool SerialUSBClass::dtr()
{
  return _lineState & 0x1;
}

bool SerialUSBClass::rts()
{
  return _lineState & 0x2;
}

SerialUSBClass SerialUSB;

#endif // USBD_USE_CDC
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

