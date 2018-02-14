#pragma once

#define PIN_POWER_LED   LED_BUILTIN

#define PIN_S_DETECT    PC5
#define PIN_S_INLEVEL   PA0
#define AIN_S_INLEVEL   PA0

#define PIN_CAN_RS      PE1

#define PIN_C_OUT_1     PA2
#define PIN_C_OUT_2     PE0

#define PIN_STATUS_GSM  PB4
#define PIN_RING_GSM    PA15
#define PIN_WAKE_GSM    PE3
#define PIN_C_PWR_GSM   PE2
#define PIN_C_KILL_GSM  PD3

//#define PIN_STANDBY_GPS n/a
#define PIN_RESET_GPS   PH3

#define PIN_C_IN1_PD    PE8
#define PIN_C_IN2_PD    PE7
#define ANALOG_VREF     2.50f

#define AIN_EXT_IN1     PA1
#define AIN_EXT_IN2     PA3
#define AIN_EXT_IN3     PC0
#define AIN_EXT_IN4     PC1

#define MODEM_VBAT      4.06f
#define MODEM_UG96      1

/* OpenTracker PINS 

#define PIN_EXT_RX      0
#define PIN_EXT_TX      1
#define PIN_S_PPS_GPS   2
#define PIN_EXT_PA21    6
#define PIN_EXT_PA20    7
#define PIN_EXT_TXD2    14
#define PIN_EXT_RXD2    15
#define PIN_TX1_GSM     16
#define PIN_RX1_GSM     17
#define PIN_TX0_GPS     18
#define PIN_RX0_GPS     19
#define PIN_EXT_SDA     20
#define PIN_EXT_SCL     21
#define PIN_EXT_IN1     28
#define PIN_EXT_IN2     29
#define PIN_EXT_PA22    30
#define PIN_EXT_PA23    31
#define PIN_CANRX       32
#define PIN_CANTX       33
#define PIN_EXT_MISO    34
#define PIN_EXT_MOSI    35
#define PIN_EXT_SCK     36
#define PIN_EXT_NS0     37
#if !defined(OPENTRACKER_HW_REV) || (OPENTRACKER_HW_REV < 0x0240)
// HW Revision 2.3 (and earlier)
#define PIN_C_REBOOT    4
#define PIN_RING_GSM    22
#define PIN_GSM_VDD_EXT 24
#define ANALOG_VREF     3.40
#define MODEM_VBAT      4.13
#define MODEM_M95       1
#else
// HW Revision 2.4 (and later)
// dropped "REBOOT" pin and "GSM VDD" (unused)
// moved "RING" to wakeup capable pins
// added "INx_PD" pull-down enable pins, to change analog input range
#define PIN_RING_GSM    24
#define PIN_C_IN1_PD    38
#define PIN_C_IN2_PD    39
#define ANALOG_VREF     3.31
#if (OPENTRACKER_HW_REV == 0x024A)
#define MODEM_VBAT      4.10
#define MODEM_UG96      1
#else
#define MODEM_VBAT      4.36
#define MODEM_M95       1
#endif
#endif

#define AIN_S_INLEVEL   49
#define AIN_EXT_IN1     28
#define AIN_EXT_IN2     29
#define AIN_EXT_PA22    30
#define AIN_EXT_PA23    31
*/

#ifndef MODEM_UG96
#define MODEM_UG96      0
#endif
#ifndef MODEM_M95
#define MODEM_M95       0
#endif
#ifndef MODEM_BG96
#define MODEM_BG96      0
#endif
