#pragma once
#include <Arduino.h>
#include "wind_packet.h"

namespace lcd_ui {

struct SettingsView {
  int16_t dir_offset_deg;   // -180..180
  float   speed_factor;     // multiplicador
  uint8_t speed_src;        // 0=PPS, 1=RPM
};

enum class UiMode : uint8_t { MAIN, MENU, EDIT };

void begin();

void renderMain(const WindPacket* p, bool ok, uint32_t age_ms, float dir_deg_corrected, float speed_value, float holdProgress = -1.0f);

// Pantalla diagnóstico (solo DIAG)
void renderDiag(const WindPacket* p, bool ok, uint32_t age_ms,
                float dir_raw_deg, float dir_corr_deg,
                float pps, float rpm, float spd,
                int16_t dir_offset_deg,
                uint32_t lost, uint32_t badLen, uint32_t badMagic, uint32_t badCrc);

// Pantalla info extra (solo INFO)
void renderInfo(const WindPacket* p, bool ok, uint32_t age_ms,
                const SettingsView& cfg,
                float dir_corr_deg, float spd);

// Pantalla menú (solo MENU/EDIT)
void renderMenu(UiMode mode, int menuIndex, const SettingsView& cfg);

} // namespace lcd_ui
