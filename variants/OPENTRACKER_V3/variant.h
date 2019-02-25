/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 */

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "PeripheralPins.h"

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
extern const PinName digitalPin[];

// Enum defining pin names to match digital pin number --> Dx
// !!!
// !!! Copy the digitalPin[] array in variant.cpp
// !!! and remove all '_': PX_n --> PXn
// !!! For NC, suffix by _x where x is the number of NC:
// !!!   NC_1, NC_2,...
// !!! For duplicated pin name, suffix by _x where x is the number of pin:
// !!! PA7, PA7_2, PA7_3,...
enum {
PE2,
PE3,
PE4,
PE5,
PE6,
PC13,
PC14,
PC15,
PH0,
PH1,
PC0,
PC1,
PC2,
PC3,
PA0,
PA1,
PA2,
PA3,
PA4,
PA5,
PA6,
PA7,
PC4,
PC5,
PB0,
PB1,
PB2,
PE7,
PE8,
PE9,
PE10,
PE11,
PE12,
PE13,
PE14,
PE15,
PB10,
PB11,
PB12,
PB13,
PB14,
PB15,
PD8,
PD9,
PD10,
PD11,
PD12,
PD13,
PD14,
PD15,
PC6,
PC7,
PC8,
PC9,
PA8,
PA9,
PA10,
PA11,
PA12,
PA13,
PA14,
PA15,
PC10,
PC11,
PC12,
PD0,
PD1,
PD2,
PD3,
PD4,
PD5,
PD6,
PD7,
PB3,
PB4,
PB5,
PB6,
PB7,
PH3,
PB8,
PB9,
PE0,
PE1,

PA0_2,
PA1_2,
PA3_2,
PA4_2,
PA5_2,
PC0_2,
PC1_2,
PC2_2,
PC3_2,
VREF,
VTEMP,
VBAT,

  PEND
};

// This must be a literal with the same value as PEND
// It is used with preprocessor tests (e.g. #if NUM_DIGITAL_PINS > 3)
// so an enum will not work.
#define NUM_DIGITAL_PINS        95

// Allow to define Arduino style alias for analog input pin number --> Ax
// All pins are digital, analog inputs are a subset of digital pins
// and must be contiguous to be able to loop on each value
// This must be a literal with a value less than or equal to MAX_ANALOG_INPUTS
// defined in pin_arduino.h
// It is used with preprocessor tests (e.g. #if NUM_ANALOG_INPUTS > 3)
// so an enum will not work.
// !!!
// !!! It must be aligned with the number of analog PinName
// !!! defined in digitalPin[] array in variant.cpp
// !!!
#define NUM_ANALOG_INPUTS       9
// Define digital pin number of the first analog input  (i.e. which digital pin is A0)
// First analog pin value (A0) must be greater than or equal to NUM_ANALOG_INPUTS
#define NUM_ANALOG_FIRST        83

//ADC sampling time
#define SAMPLINGTIME ADC_SAMPLETIME_640CYCLES_5

//ADC resolution is 12bits
#define ADC_RESOLUTION          12
#define DACC_RESOLUTION         12

//PWM resolution
#define PWM_RESOLUTION          8
#define PWM_FREQUENCY           1000
#define PWM_MAX_DUTY_CYCLE      255

//On-board LED pin number
#define LED_BUILTIN             PB2

//On-board user button
#define USER_BTN                PE6 // SOS input


//SPI definitions
#define PIN_SPI_SS              PE12 // OUT driver
#define PIN_SPI_SS1             PD10 // Accelerometer
#define PIN_SPI_MOSI            PE15
#define PIN_SPI_MISO            PE14
#define PIN_SPI_SCK             PE13

//I2C Definitions
#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

//Timer Definitions
//Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM6
#define TIMER_UART_EMULATED     TIM7

//Do not use basic timer: OC is required
#define TIMER_SERVO             TIM2 //TODO: advanced-control timers don't work

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#ifdef USBD_USE_CDC
#define Serial SerialUSB
#else
#define SERIAL_UART_INSTANCE    3 //ex: 2 for Serial2 (USART2)
// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           PD9
#define PIN_SERIAL_TX           PD8
#endif

// DEBUG_UART could be redefined to print on another instance than 'Serial'
//#define DEBUG_UART              ((USART_TypeDef *) USART3) // ex: USART3
#define DEBUG_UART              NP
#define DEBUG_PINNAME_TX        NC

// UART Emulation (uncomment if needed, required TIM1)
//#define UART_EMUL_RX            PX_n // PinName used for RX
//#define UART_EMUL_TX            PX_n // PinName used for TX

#ifdef __cplusplus
} // extern "C"
#endif
/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR     Serial
#define SERIAL_PORT_HARDWARE    Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial11
#define SERIAL_PORT_HARDWARE_OPEN1  Serial3
#endif

#include "opentracker.h"

#ifdef __cplusplus
void analogWritePWM(uint32_t ulPin, uint32_t ulValue, uint32_t ulMax, uint32_t ulFreq);
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
