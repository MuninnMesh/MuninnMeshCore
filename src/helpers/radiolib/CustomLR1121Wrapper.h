#pragma once

#include "CustomLR1121.h"
#include "RadioLibWrappers.h"
#include "LR11x0Reset.h"

class CustomLR1121Wrapper : public RadioLibWrapper {
public:
  CustomLR1121Wrapper(CustomLR1121& radio, mesh::MainBoard& board) : RadioLibWrapper(radio, board) { }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomLR1121 *)_radio)->setFrequency(freq);
    ((CustomLR1121 *)_radio)->setSpreadingFactor(sf);
    ((CustomLR1121 *)_radio)->setBandwidth(bw);
    ((CustomLR1121 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
  }

  void doResetAGC() override { lr11x0ResetAGC((LR11x0 *)_radio, ((CustomLR1121 *)_radio)->getFreqMHz()); }
  bool isReceivingPacket() override {
    return ((CustomLR1121 *)_radio)->isReceiving();
  }

  bool isRxDonePending() override {
    // Backstop for a missed DIO edge: RX_DONE latched in the IRQ status while
    // the wrapper still thinks it is waiting (see RadioLibWrapper::loop()).
    return (((CustomLR1121 *)_radio)->getIrqStatus() & RADIOLIB_LR11X0_IRQ_RX_DONE) != 0;
  }
  float getCurrentRSSI() override {
    float rssi = -110;
    ((CustomLR1121 *)_radio)->getRssiInst(&rssi);
    return rssi;
  }

  void onSendFinished() override {
    RadioLibWrapper::onSendFinished();
    // Keep LR1121 preamble length aligned with MeshCore's current SF after TX;
    // the radio object may have packet-length-specific state after send.
    _radio->setPreambleLength(preambleLengthForSF(getSpreadingFactor()));
  }

  float getLastRSSI() const override { return ((CustomLR1121 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomLR1121 *)_radio)->getSNR(); }

  uint8_t getSpreadingFactor() const override { return ((CustomLR1121 *)_radio)->getSpreadingFactor(); }

  uint16_t getRxLatchClears() const override { return ((CustomLR1121 *)_radio)->getLatchClears(); }

  void setRxBoostedGainMode(bool en) override {
    ((CustomLR1121 *)_radio)->setRxBoostedGainMode(en);
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomLR1121 *)_radio)->getRxBoostedGainMode();
  }
};
