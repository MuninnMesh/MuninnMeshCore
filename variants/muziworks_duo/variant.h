/*
 * variant.h - MuziWorks Base family
 * nRF52840 + LR1121 Base Duo or SX1262 Base Uno.
 */

#pragma once

#include "WVariant.h"

#if defined(MUZI_BASE_DUO) && !defined(MUZIWORKS_DUO)
#define MUZIWORKS_DUO
#endif

#if defined(MUZI_BASE_UNO) && !defined(MUZIWORKS_UNO)
#define MUZIWORKS_UNO
#endif

#if defined(MUZI_BASE_DUO_SUPER_IO) && !defined(MUZIWORKS_DUO_SUPER_IO)
#define MUZIWORKS_DUO_SUPER_IO
#endif

#if defined(MUZI_BASE_UNO_SUPER_IO) && !defined(MUZIWORKS_UNO_SUPER_IO)
#define MUZIWORKS_UNO_SUPER_IO
#endif

// The Super IO connector and peripheral pins are shared by Base Uno and Base Duo.
#if defined(MUZIWORKS_UNO_SUPER_IO) && !defined(MUZIWORKS_DUO_SUPER_IO)
#define MUZIWORKS_DUO_SUPER_IO
#endif

////////////////////////////////////////////////////////////////////////////////
// Low frequency clock source

#define USE_LFXO    // 32.768 kHz crystal oscillator
#define VARIANT_MCK (64000000ul)

////////////////////////////////////////////////////////////////////////////////
// Power

#define BATTERY_PIN             (31)            // P0.31 (AIN7)
#define BATTERY_IMMUTABLE
#define ADC_MULTIPLIER          (1.537F)
#define AREF_VOLTAGE            (3.0F)

#define ADC_RESOLUTION          (12)
#define BATTERY_SENSE_RES       (12)

#define PIN_BATTERY_CHARGING    (34)            // P1.02 / BQ25185 STAT2, LOW while charging
#define PIN_CHARGER_FAULT       (27)            // P0.27 / BQ25185 STAT1

#define PWRMGT_VOLTAGE_BOOTLOCK 3100            // Do not boot on battery below this voltage (mV)
#define PWRMGT_LPCOMP_AIN       7               // AIN7 = P0.31 = BATTERY_PIN
#define PWRMGT_LPCOMP_REFSEL    4               // 5/8 VDD wake threshold

////////////////////////////////////////////////////////////////////////////////
// Number of pins

#define PINS_COUNT              (48)
#define NUM_DIGITAL_PINS        (48)
#define NUM_ANALOG_INPUTS       (8)
#define NUM_ANALOG_OUTPUTS      (0)

////////////////////////////////////////////////////////////////////////////////
// UART pin definition

#define PIN_SERIAL1_RX          (20)            // P0.20 (GPS RX)
#define PIN_SERIAL1_TX          (19)            // P0.19 (GPS TX)

////////////////////////////////////////////////////////////////////////////////
// I2C pin definition

#define HAS_WIRE                (1)
#define WIRE_INTERFACES_COUNT   (2)

#define PIN_WIRE_SDA            (24)            // P0.24
#define PIN_WIRE_SCL            (25)            // P0.25
#define PIN_WIRE1_SDA           (4)             // P0.04, secondary I2C bus for future IMU/RTC
#define PIN_WIRE1_SCL           (6)             // P0.06, secondary I2C bus for future IMU/RTC
#define I2C_NO_RESCAN

////////////////////////////////////////////////////////////////////////////////
// SPI pin definition (internal to the Elecrow radio module)

#define SPI_INTERFACES_COUNT    (1)

#define PIN_SPI_MISO            (47)            // P1.15
#define PIN_SPI_MOSI            (46)            // P1.14
#define PIN_SPI_SCK             (45)            // P1.13
#define PIN_SPI_NSS             (44)            // P1.12

////////////////////////////////////////////////////////////////////////////////
// Builtin LEDs (active LOW)

#define LED_GREEN               (35)            // P1.03
#define LED_BLUE                (36)            // P1.04
#define LED_BUILTIN             LED_GREEN
#define LED_PIN                 LED_GREEN

#define LED_STATE_ON            LOW

////////////////////////////////////////////////////////////////////////////////
// Builtin buttons

#define PIN_BUTTON1             (-1)
#define BUTTON_PIN              PIN_BUTTON1

////////////////////////////////////////////////////////////////////////////////
// Radio pins. Base Duo uses LR1121 IRQ on P1.08; Base Uno uses SX1262 DIO1 on P1.06.

#if defined(MUZIWORKS_UNO)
#define LORA_DIO_1              (38)            // P1.06 - SX1262 DIO1
#else
#define LORA_DIO_1              (40)            // P1.08 - LR1121 IRQ (DIO1)
#endif
#define LORA_NSS                (PIN_SPI_NSS)   // P1.12
#define LORA_RESET              (42)            // P1.10 - radio reset
#define LORA_BUSY               (43)            // P1.11 - radio BUSY
#define LORA_SCLK               (PIN_SPI_SCK)   // P1.13
#define LORA_MISO               (PIN_SPI_MISO)  // P1.15
#define LORA_MOSI               (PIN_SPI_MOSI)  // P1.14
#define LORA_CS                 PIN_SPI_NSS     // P1.12

#if defined(MUZIWORKS_UNO)
#define SX126X_DIO2_AS_RF_SWITCH    true
#define SX126X_DIO3_TCXO_VOLTAGE    3.3
#else
#define LR11X0_DIO_AS_RF_SWITCH    true
#define LR11X0_DIO3_TCXO_VOLTAGE   3.0
#endif

////////////////////////////////////////////////////////////////////////////////
// GPS/GNSS

#if defined(MUZIWORKS_DUO_SUPER_IO)
#define HAS_GPS                 1
#define PIN_GPS_TX              PIN_SERIAL1_RX    // GPS module TX -> MCU RX (P0.20)
#define PIN_GPS_RX              PIN_SERIAL1_TX    // MCU TX -> GPS module RX (P0.19)
#define GPS_EN                  (33)              // P1.01
#define PIN_GPS_EN              GPS_EN
#define GPS_RESET               (-1)
#define PIN_GPS_SWITCH          (12)              // P0.12 / SWITCH_MODE2
#else
#define HAS_GPS                 0
#define PIN_GPS_TX              (-1)
#define PIN_GPS_RX              (-1)
#define GPS_EN                  (-1)
#define GPS_RESET               (-1)
#endif

////////////////////////////////////////////////////////////////////////////////
// QSPI Flash

#define PIN_QSPI_SCK            (3)             // P0.03
#define PIN_QSPI_CS             (26)            // P0.26
#define PIN_QSPI_IO0            (30)            // P0.30
#define PIN_QSPI_IO1            (29)            // P0.29
#define PIN_QSPI_IO2            (28)            // P0.28
#define PIN_QSPI_IO3            (2)             // P0.02

#if defined(MUZIWORKS_UNO)
#define EXTERNAL_FLASH_DEVICES  W25Q32JVSS      // Base Uno: 2MB
#else
#define EXTERNAL_FLASH_DEVICES  W25Q128JVPQ     // Base Duo: 16MB
#endif
#define EXTERNAL_FLASH_USE_QSPI

////////////////////////////////////////////////////////////////////////////////
// Additional peripheral pins (not enabled)
//
// I2C bus 1 (IMU):       SDA P0.04, SCL P0.06
// IMU/Compass:           ICM-20948 on secondary I2C bus, yaw rotation 270 deg
// RTC:                   RX8130CE expected at I2C address 0x32 on Wire or Wire1
// Display (SH1107 OLED): I2C bus 0, power enable P0.23
// Trackball:             UP P0.21, DOWN P0.17, LEFT P1.05, RIGHT P0.16, PRESS P0.10
// Buzzer:                P0.22
// Buttons:               Cancel P0.15, Mode1 P1.09, Mode2 P0.12
// GPS:                   RX P0.20, TX P0.19, EN P1.01
// Charge status:         P1.02 (inverted)

#if defined(MUZIWORKS_DUO_SUPER_IO)
#define PIN_SCREEN_ENABLE       (23)              // P0.23
#define SCREEN_12V_ENABLE       PIN_SCREEN_ENABLE
#define SCREEN_ENABLE_ACTIVE    HIGH              // PR2054 hardware test: HIGH enables the SH1107 rail

#define JOYSTICK_UP             (21)              // P0.21
#define JOYSTICK_DOWN           (17)              // P0.17
#define JOYSTICK_LEFT           (37)              // P1.05
#define JOYSTICK_RIGHT          (16)              // P0.16
#define JOYSTICK_PRESS          (10)              // P0.10
#define PIN_USER_BTN            JOYSTICK_PRESS
#define PIN_BACK_BTN            (15)              // P0.15

#define SWITCH_MODE1            (41)              // P1.09
#define SWITCH_MODE2            (12)              // P0.12
#define MUZIWORKS_SWITCH_ACTIVE LOW
#ifndef MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS
#define MUZIWORKS_SUPER_IO_USE_INTERNAL_PULLUPS 1
#endif

#if defined(MUZIWORKS_SUPER_IO_ACTIVE_BUZZER)
#define PIN_ACTIVE_BUZZER       (22)              // P0.22
#ifndef ACTIVE_BUZZER_ON
#define ACTIVE_BUZZER_ON        HIGH
#endif
#elif !defined(MUZIWORKS_SUPER_IO_DISABLE_BUZZER)
#define PIN_BUZZER              (22)              // P0.22
#endif
#define PIN_STATUS_LED          LED_GREEN
#define PIN_MSG_LED             LED_BLUE
#endif
