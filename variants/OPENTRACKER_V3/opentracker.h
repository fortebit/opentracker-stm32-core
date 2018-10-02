#pragma once

#define PIN_POWER_LED   LED_BUILTIN

#define PIN_S_DETECT    PC5
#define PIN_S_INLEVEL   PA0
#define AIN_S_INLEVEL   PA0

#define PIN_CAN_RS      PE1

#define PIN_C_OUT_1     PA2
#define PIN_C_OUT_2     PE0

#define PIN_STATUS_GSM  PB4
#define PIN_WAKE_GSM    PE3
#define PIN_C_PWR_GSM   PE2
#define PIN_C_KILL_GSM  PD3

//#define PIN_S_PPS_GPS   NC

#define PIN_C_IN1_PD    PE8
#define PIN_C_IN2_PD    PE7
#define PIN_C_IN3_PD    PD14
#define PIN_C_IN4_PD    PD12

#define ANALOG_VREF     2.50f
#define ANALOG_SCALE    13.0154525f
#define ANALOG_SCALE_LOW 2.0f

#define AIN_EXT_IN1     PA1
#define AIN_EXT_IN2     PA3
#define AIN_EXT_IN3     PC0
#define AIN_EXT_IN4     PC1

#define AIN_EXT_AN2     PC2
#define AIN_EXT_AN1     PC3

#define PIN_EXT_IN1     PA1
#define PIN_EXT_IN2     PA3
#define PIN_EXT_IN3     PC0
#define PIN_EXT_IN4     PC1

#define PIN_EXT_AN2     PC2
#define PIN_EXT_AN1     PC3

#define PIN_EXT_TXD2    PD8
#define PIN_EXT_RXD2    PD9
#define PIN_EXT_TX      PB11
#define PIN_EXT_RX      PB10
#define PIN_TX1_GSM     PD5
#define PIN_RX1_GSM     PD6
#define PIN_TX0_GPS     PA9
#define PIN_RX0_GPS     PA10

#define PIN_EXT_SDA     PB7
#define PIN_EXT_SCL     PB6

#define PIN_CANRX       PB8
#define PIN_CANTX       PB9

#define PIN_EXT_MISO    PB14
#define PIN_EXT_MOSI    PB15
#define PIN_EXT_SCK     PB13
#define PIN_EXT_NS0     PB12

#define PIN_S_BUTTON    PE6
#define PIN_C_POWERMODE PE4
#define PIN_C_5V_ENABLE PE5
#define PIN_EXT_AUDIO   PD4
#define PIN_C_ANTON     PD0
#define PIN_S_USBVBUS   PC7

#define PIN_EXT_INT     PD15
#define PIN_EXT_RST     PC4
#define PIN_EXT_DAC1    PA4
#define PIN_EXT_DAC2    PA5

#if OPENTRACKER_HW_REV > 0x0300
#define PIN_EXT_GPIO1   PD13
#define PIN_EXT_GPIO2   PB5
#define PIN_EXT_PWM     PA15

#define PIN_RING_GSM    PD1

#define PIN_RESET_GPS   PD7
#define PIN_STANDBY_GPS PC6
#else
#define PIN_EXT_GPIO1   PB5
#define PIN_EXT_GPIO2   PD7
#define PIN_EXT_PWM     PD13

#define PIN_RING_GSM    PA15

#define PIN_RESET_GPS   PH3
#define PIN_C_OUT_ENA   PC6
#endif

#define PIN_S_ACC_INT1  PD11

#define PIN_C_ACC_CS    PD10
#define PIN_C_OUT_CS    PE12

#define MODEM_VBAT      4.10f

#ifndef MODEM_UG96
#define MODEM_UG96      0
#endif
#ifndef MODEM_M95
#define MODEM_M95       0
#endif
#ifndef MODEM_BG96
#define MODEM_BG96      0
#endif
#ifndef MODEM_EG91
#define MODEM_EG91      0
#endif
