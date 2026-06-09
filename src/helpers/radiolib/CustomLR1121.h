#pragma once

#include <RadioLib.h>
#include "MeshCore.h"

class CustomLR1121 : public LR1121 {
  bool _rx_boosted = false;

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
      uint16_t irq = getIrqStatus();
      // Treat preamble or valid sync/header IRQs as "channel busy receiving"
      // for UI/scanner status without waiting for a complete packet.
      bool detected = ((irq & RADIOLIB_LR11X0_IRQ_SYNC_WORD_HEADER_VALID) || (irq & RADIOLIB_LR11X0_IRQ_PREAMBLE_DETECTED));
      return detected;
    }

    uint8_t getSpreadingFactor() const { return spreadingFactor; }
};
