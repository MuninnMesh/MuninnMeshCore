#pragma once

#include <RadioLib.h>
#include "MeshCore.h"

// How long isReceiving() may report "busy" from latched preamble/header IRQs
// before treating them as stale and clearing them. Must comfortably exceed the
// airtime of a legitimate in-flight packet, and stay below the Dispatcher's
// 4s CAD give-up window. 2500ms suits SF7/BW62.5 (max frame airtime ~0.85s,
// ~3x margin). WARNING: a build at high SF on a narrow BW (e.g. SF12/BW62.5,
// ~5-20s airtime) MUST raise this, or the latch can clear mid-legit-reception.
#ifndef LR1121_RX_BUSY_LATCH_MS
  #define LR1121_RX_BUSY_LATCH_MS 2500
#endif

class CustomLR1121 : public LR1121 {
  bool _rx_boosted = false;
  uint32_t _rx_busy_since = 0;
  uint16_t _n_latch_clears = 0;

  public:
    CustomLR1121(Module *mod) : LR1121(mod) { }

    size_t getPacketLength(bool update) override {
      size_t len = LR1121::getPacketLength(update);
      if (len == 0 && getIrqStatus() & RADIOLIB_LR11X0_IRQ_HEADER_ERR) {
        // Clear the RX engine after a bad LoRa header so the next receive path
        // does not inherit a stale LR11x0 IRQ state.
        MESH_DEBUG_PRINTLN("LR1121: got header err, calling standby()");
        standby();
      }
      return len;
    }

    float getFreqMHz() const { return freqMHz; }

    int16_t setRxBoostedGainMode(bool en) {
      // RadioLib exposes the setter but not a stable getter; cache the desired
      // state for diagnostics/UI.
      _rx_boosted = en;
      return LR1121::setRxBoostedGainMode(en);
    }

    bool getRxBoostedGainMode() const { return _rx_boosted; }

    bool isReceiving() {
      uint32_t irq = getIrqStatus();
      if (irq & RADIOLIB_LR11X0_IRQ_RX_DONE) {
        return true;   // complete packet waiting to be read out
      }
      // Preamble or valid sync/header IRQs mean "channel busy receiving".
      bool detected = ((irq & RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID) || (irq & RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED));
      if (!detected) {
        _rx_busy_since = 0;
        return false;
      }
      // These IRQs LATCH until explicitly cleared. A preamble detected from
      // noise, or a frame whose header fails, never reaches RX_DONE, so
      // readData() never runs and the flags stay stuck — which makes the
      // Dispatcher defer every outbound TX by its CAD backoff (up to 4s per
      // packet). On a busy mesh that means chronically late replies. Bound the
      // latch: clear stale preamble/header flags if no packet completes within
      // the window (clearing IRQ status does not disturb an actual in-progress
      // demodulation).
      uint32_t now_ms = millis();
      if (_rx_busy_since == 0) {
        _rx_busy_since = now_ms;
      } else if (now_ms - _rx_busy_since > LR1121_RX_BUSY_LATCH_MS) {
        MESH_DEBUG_PRINTLN("LR1121: clearing stale preamble/header IRQ latch");
        clearIrqState(RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED
                      | RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID
                      | RADIOLIB_LR11X0_IRQ_HEADER_ERR);
        _n_latch_clears++;
        _rx_busy_since = 0;
        return false;
      }
      return true;
    }

    uint16_t getLatchClears() const { return _n_latch_clears; }

    uint8_t getSpreadingFactor() const { return spreadingFactor; }
};
