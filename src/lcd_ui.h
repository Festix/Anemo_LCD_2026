#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "wind_packet.h"

namespace lcd_ui {

struct SettingsView {
  int16_t dir_offset_deg;
  float   speed_factor;
  uint8_t speed_src;
  uint8_t espnow_channel;   // 1..13
  const char* macStr;       // "AA:BB:CC:DD:EE:FF"
};

enum class UiMode : uint8_t { MAIN, MENU, EDIT };

void begin();
void renderMain(const WindPacket* p, bool ok, uint32_t age_ms,
                float dir_deg_corrected, float speed_value,
                float holdProgress = -1.0f);

void renderDiag(const WindPacket* p, bool ok, uint32_t age_ms,
                uint32_t seq, uint16_t status,
                const char* macStr,
                uint32_t badLen, uint32_t badMagic, uint32_t badCrc);

void renderMenu(UiMode mode, int menuIndex, const SettingsView& cfg);

} // namespace lcd_ui
