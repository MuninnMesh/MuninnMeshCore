#include <Arduino.h>
#include "target.h"

XiaoC6Board board;

#if defined(P_LORA_SCLK)
  #if defined(FSPI)
    static SPIClass spi(FSPI);
  #else
    static SPIClass spi(0);
  #endif
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
#else
  RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY);
#endif

WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
EnvironmentSensorManager sensors;

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true);
#endif

#ifndef LORA_CR
  #define LORA_CR 5
#endif

#ifndef RADIO_INIT_ATTEMPTS
  #define RADIO_INIT_ATTEMPTS 3   // re-init (and re-reset) the LR1121 this many
                                  // times before giving up, instead of spinning.
#endif

#ifndef DISABLE_I2C_INIT
  #define DISABLE_I2C_INIT 0
#endif

#if defined(PIN_STATUS_LED) && !defined(PIN_STATUS_LED_ON)
  #define PIN_STATUS_LED_ON LOW
#endif

static void statusLedSet(bool on) {
#ifdef PIN_STATUS_LED
  digitalWrite(PIN_STATUS_LED, on ? PIN_STATUS_LED_ON : !PIN_STATUS_LED_ON);
#else
  (void)on;
#endif
}

static void statusLedBlink(uint8_t count, uint16_t on_ms = 80, uint16_t off_ms = 80) {
#ifdef PIN_STATUS_LED
  for (uint8_t i = 0; i < count; i++) {
    statusLedSet(true);
    delay(on_ms);
    statusLedSet(false);
    delay(off_ms);
  }
#else
  (void)count;
  (void)on_ms;
  (void)off_ms;
#endif
}

static void i2c_init() {
#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL) && PIN_BOARD_SDA >= 0 && PIN_BOARD_SCL >= 0
  Wire.begin(PIN_BOARD_SDA, PIN_BOARD_SCL);
#else
  Wire.begin();
#endif
  Wire.setClock(100000);
  Wire.setTimeOut(10);
}

// NiceRF LoRa1121F33-2G4 RF-switch wiring (module-internal, driven by LR1121
// DIOs; the module exposes no RXEN/TXEN pins). Sub-GHz TXEN = DIO6, asserted
// only while transmitting — it keys the 1W PA and the antenna switch. The
// sub-GHz RX path is passive (no enable line), so MODE_RX drives nothing.
// DIO5/DIO8 belong to the 2.4G FEM (RX/TX), unused on this US915 node but
// pinned LOW so the FEM can never wake up. Same table as the NiceRF F33
// demo code (LF TX asserted only).
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,  {LOW,  LOW,  LOW,  LOW }},
  { LR11x0::MODE_RX,    {LOW,  LOW,  LOW,  LOW }},   // sub-GHz RX is passive on the F33
  { LR11x0::MODE_TX,    {LOW,  HIGH, LOW,  LOW }},   // DIO6 = sub-GHz TXEN (PA key)
  { LR11x0::MODE_TX_HP, {LOW,  HIGH, LOW,  LOW }},   // HP PA path — same TXEN
  { LR11x0::MODE_TX_HF, {LOW,  LOW,  LOW,  HIGH}},   // 2.4G TX (never selected at 915MHz)
  { LR11x0::MODE_GNSS,  {LOW,  LOW,  LOW,  LOW }},
  { LR11x0::MODE_WIFI,  {LOW,  LOW,  LOW,  LOW }},
  END_OF_MODE_TABLE,
};

bool radio_init() {
#ifdef PIN_STATUS_LED
  pinMode(PIN_STATUS_LED, OUTPUT);
  statusLedSet(false);
#endif
  statusLedBlink(2);

  // Bring the LoRa front-end up FIRST; I2C sensor/RTC discovery runs after
  // radio.begin() so a flaky or unpowered I2C bus can never delay or block
  // the radio coming up.

#ifdef P_LORA_EN
  // LoRa1121F33 CE (pin 5): module-internal LDO enable, internally pulled up,
  // 0-5.5V tolerant. Drive it high before any SPI traffic; driving it low is
  // the module's sleep mode.
  pinMode(P_LORA_EN, OUTPUT);
  digitalWrite(P_LORA_EN, HIGH);
  delay(10);
#endif

#if defined(P_LORA_SCLK)
  pinMode(P_LORA_NSS, OUTPUT);
  digitalWrite(P_LORA_NSS, HIGH);   // deselect before the bus is brought up
  spi.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
#endif

#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 3.3f;   // NiceRF F33 modules run their 32MHz TCXO from VTCXO at 3.3V
#endif

  // Retry the whole init a few times (each begin() re-resets the chip) so a
  // single transient at cold boot recovers instead of bricking the node.
  int status = RADIOLIB_ERR_NONE;
  bool ok = false;
  for (int attempt = 1; attempt <= RADIO_INIT_ATTEMPTS && !ok; attempt++) {
    status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                         RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
    ok = (status == RADIOLIB_ERR_NONE);
    if (!ok) {
      MESH_DEBUG_PRINTLN("LR1121 init attempt %d failed, status=%d", attempt, status);
      statusLedBlink(3, 60, 60);
      delay(50);
    }
  }

  if (!ok) {
    // Do NOT spin forever here (that bricks boot and starves the watchdog).
    // Leave a short SOS on the LED and return false so main can halt() with a log.
    MESH_DEBUG_PRINTLN("LR1121 init failed after retries, status=%d (-2=chip not found: SPI/GND/CE; -706/-707=TCXO)", status);
    statusLedBlink(6, 120, 120);
    return false;
  }

  // The RF-switch table must be programmed before the first TX: without it the
  // PA is never keyed and TX goes into an idle switch.
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
  radio.setCRC(2);
  radio.explicitHeader();
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  statusLedSet(true);  // solid ON only after the radio is up

  // ---- Radio is up. Now do the non-critical I2C clock/RTC discovery. ----
#if defined(DISABLE_I2C_INIT) && DISABLE_I2C_INIT
  fallback_clock.begin();
#else
  i2c_init();
  fallback_clock.begin();
  rtc_clock.begin(Wire);
#endif

  return true;
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
  radio_driver.updatePreamble(sf);
}

void radio_set_tx_power(int8_t dbm) {
#ifdef MAX_LORA_TX_POWER
  // LR1121 HP PA tops out at chip +22 (module ~+30.8dBm at 915MHz). RadioLib
  // would reject higher values with an error and silently keep the old power;
  // clamp instead so a stale saved pref can never leave the radio mis-set.
  if (dbm > MAX_LORA_TX_POWER) dbm = MAX_LORA_TX_POWER;
#endif
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);
}
