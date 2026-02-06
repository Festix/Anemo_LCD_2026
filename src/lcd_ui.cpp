#include "lcd_ui.h"
#include <U8g2lib.h>
#include <math.h>
#include "config.h"

static U8G2_ST7920_128X64_F_SW_SPI u8g2(
  U8G2_R0,
  /* clock=*/ LCD_CLK,
  /* data=*/  LCD_DAT,
  /* CS=*/    LCD_CS,
  /* reset=*/ LCD_RST
);

static inline float deg2rad(float d){ return d * 3.14159265359f / 180.0f; }

namespace {

constexpr int CX=25, CY=25, R=24;

void drawCompass(float deg, bool valid) {
  u8g2.drawCircle(CX, CY, R);
  u8g2.drawVLine(CX, CY-R, 4); u8g2.drawVLine(CX, CY+R-4, 4);
  u8g2.drawHLine(CX-R, CY, 4); u8g2.drawHLine(CX+R-4, CY, 4);

  if (!valid) {
    u8g2.drawLine(CX-6, CY-6, CX+6, CY+6);
    u8g2.drawLine(CX-6, CY+6, CX+6, CY-6);
    return;
  }

  float a = deg2rad(deg - 90.0f);
  int x2 = CX + (int)(cosf(a) * (R - 3));
  int y2 = CY + (int)(sinf(a) * (R - 3));
  u8g2.drawLine(CX, CY, x2, y2);
  u8g2.drawDisc(CX, CY, 2);
}

const char* menuLabel(int idx) {
  switch (idx) {
    case 0: return "Offset proa";
    case 1: return "Factor vel.";
    case 2: return "Fuente vel.";
    default: return "";
  }
}

} // namespace

namespace lcd_ui {

void begin() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "ANEMO RX");
  u8g2.drawStr(0, 28, "ST7920 + ESP-NOW");
  u8g2.drawStr(0, 44, "Boot...");
  u8g2.sendBuffer();
}

void renderMain(const WindPacket* p, bool ok, uint32_t age_ms,
                float dir_deg_corrected, float speed_value, float holdProgress)
{
  u8g2.clearBuffer();

  // --- Layout (128x64) ---
  const int cx = 30;
  const int cy = 32;
  const int r  = 26;
  const int xText = 62;

  // Validez de dirección (según tu status bit1 AS5600 OK)
  bool validDir = ok && p && ((p->status & (1u << 1)) != 0);

  // ===== Rosa / compás grande =====
  u8g2.drawCircle(cx, cy, r);
  u8g2.drawCircle(cx, cy, r - 1);

  // Marcas cardinales simples (N/E/S/O)
  u8g2.drawVLine(cx, cy - r, 4);        // N
  u8g2.drawVLine(cx, cy + r - 3, 4);    // S
  u8g2.drawHLine(cx - r, cy, 4);        // W
  u8g2.drawHLine(cx + r - 3, cy, 4);    // E

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(cx - 2, cy - r - 1, "N");

  if (!validDir) {
    // cruz cuando no hay dirección válida
    u8g2.drawLine(cx - 10, cy - 10, cx + 10, cy + 10);
    u8g2.drawLine(cx - 10, cy + 10, cx + 10, cy - 10);
  } else {
    // Flecha: 0° = Norte (arriba). Convertimos a radianes y rotamos -90°
    float a = deg2rad(dir_deg_corrected - 90.0f);
    int x2 = cx + (int)(cosf(a) * (r - 4));
    int y2 = cy + (int)(sinf(a) * (r - 4));

    u8g2.drawLine(cx, cy, x2, y2);
    u8g2.drawDisc(cx, cy, 2);

    // puntita de flecha
    int x3 = cx + (int)(cosf(a + 0.25f) * (r - 8));
    int y3 = cy + (int)(sinf(a + 0.25f) * (r - 8));
    int x4 = cx + (int)(cosf(a - 0.25f) * (r - 8));
    int y4 = cy + (int)(sinf(a - 0.25f) * (r - 8));
    u8g2.drawLine(x2, y2, x3, y3);
    u8g2.drawLine(x2, y2, x4, y4);
  }

  // ===== Textos a la derecha =====
  char b[32];

  // Labels
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(xText, 12, "DIR");
  u8g2.drawStr(xText, 36, "SPD");

  // DIR grande
  u8g2.setFont(u8g2_font_7x13B_tf);
  if (ok && p) snprintf(b, sizeof(b), "%.1f%c", dir_deg_corrected, 176);
  else        snprintf(b, sizeof(b), "--.-%c", 176);
  u8g2.drawStr(xText, 26, b);

  // SPD grande
  u8g2.setFont(u8g2_font_7x13B_tf);
  if (ok && p) snprintf(b, sizeof(b), "%.2f", speed_value);
  else        snprintf(b, sizeof(b), "--.--");
  u8g2.drawStr(xText, 50, b);

  // ===== Pie: barra de hold o estado =====
  u8g2.setFont(u8g2_font_5x8_tf);

  if (holdProgress >= 0.0f) {
    if (holdProgress > 1.0f) holdProgress = 1.0f;

    const int x = 0, y = 56, w = 128, h = 8;
    u8g2.drawFrame(x, y, w, h);

    int fill = (int)((w - 2) * holdProgress);
    if (fill < 0) fill = 0;
    if (fill > (w - 2)) fill = (w - 2);

    u8g2.drawBox(x + 1, y + 1, fill, h - 2);
  } else {
    // Estado normal: OK / SIN DATOS + age
    if (!ok || !p) {
      u8g2.drawStr(0, 63, "SIN DATOS");
    } else {
      snprintf(b, sizeof(b), "OK %lums", (unsigned long)age_ms);
      u8g2.drawStr(0, 63, b);
    }
  }

  u8g2.sendBuffer();
}


void renderDiag(const WindPacket* p, bool ok, uint32_t age_ms,
                uint32_t seq, uint16_t status,
                const char* macStr,
                uint32_t badLen, uint32_t badMagic, uint32_t badCrc)
{
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "DIAG");

  u8g2.setFont(u8g2_font_5x8_tf);
  char b[32];

  // Línea 1: link
  if (!ok || !p) {
    u8g2.drawStr(0, 24, "LINK: OFFLINE");
  } else {
    u8g2.drawStr(0, 24, "LINK: ONLINE");
  }

  // Línea 2: SEQ
  snprintf(b, sizeof(b), "SEQ: %lu", (unsigned long)seq);
  u8g2.drawStr(0, 34, b);

  // Línea 3: AGE
  snprintf(b, sizeof(b), "AGE: %lums", (unsigned long)age_ms);
  u8g2.drawStr(0, 44, b);

  // Línea 4: STATUS
  snprintf(b, sizeof(b), "STAT: 0x%04X", (unsigned)status);
  u8g2.drawStr(0, 54, b);

  // Línea 5: MAC (abajo)
  if (macStr && macStr[0]) {
    // “MAC: xx:xx:...”
    char m[32];
    snprintf(m, sizeof(m), "MAC: %s", macStr);
    u8g2.drawStr(0, 63, m);
  } else {
    // fallback: contadores mínimos
    snprintf(b, sizeof(b), "badL:%lu badM:%lu badC:%lu",
             (unsigned long)badLen, (unsigned long)badMagic, (unsigned long)badCrc);
    u8g2.drawStr(0, 63, b);
  }

  u8g2.sendBuffer();
}

void renderInfo(const WindPacket* p, bool ok, uint32_t age_ms,
                const SettingsView& cfg,
                float dir_corr_deg, float spd) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "INFO");

  u8g2.setFont(u8g2_font_5x8_tf);
  char b[44];

  if (!ok || !p) {
    u8g2.drawStr(0, 26, "SIN DATOS");
    snprintf(b, sizeof(b), "Offset: %d deg", (int)cfg.dir_offset_deg);
    u8g2.drawStr(0, 40, b);
    snprintf(b, sizeof(b), "Factor: x%.3f", cfg.speed_factor);
    u8g2.drawStr(0, 50, b);
    snprintf(b, sizeof(b), "Fuente: %s", (cfg.speed_src==0)?"PPS":"RPM");
    u8g2.drawStr(0, 60, b);
    u8g2.sendBuffer();
    return;
  }

  snprintf(b, sizeof(b), "age:%lums  seq:%lu", (unsigned long)age_ms, (unsigned long)p->seq);
  u8g2.drawStr(0, 26, b);
  snprintf(b, sizeof(b), "Dir: %.1f%c", dir_corr_deg, 176);
  u8g2.drawStr(0, 38, b);
  snprintf(b, sizeof(b), "Spd: %.2f", spd);
  u8g2.drawStr(0, 50, b);
  snprintf(b, sizeof(b), "Off:%d  x%.3f %s",
           (int)cfg.dir_offset_deg, cfg.speed_factor, (cfg.speed_src==0)?"PPS":"RPM");
  u8g2.drawStr(0, 62, b);

  u8g2.sendBuffer();
}

void renderMenu(UiMode mode, int menuIndex, const SettingsView& cfg) {
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(6, 14, (mode == UiMode::EDIT) ? "CONFIG (EDIT)" : "CONFIG");

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(6, 32, menuLabel(menuIndex));

  char v[32];
  if (menuIndex == 0) snprintf(v, sizeof(v), "%d%c", (int)cfg.dir_offset_deg, 176);
  else if (menuIndex == 1) snprintf(v, sizeof(v), "x%.3f", cfg.speed_factor);
  else if (menuIndex == 2) snprintf(v, sizeof(v), "%s", (cfg.speed_src==0) ? "PPS" : "RPM");
  else snprintf(v, sizeof(v), "-");

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(6, 54, v);

  // Ayuda corta (para que se lea)
  u8g2.setFont(u8g2_font_5x8_tf);
  if (mode == UiMode::EDIT) u8g2.drawStr(6, 63, "B2+  B3-   OK=GUARDA  B1=SALIR");
  else                     u8g2.drawStr(6, 63, "B2/B3 ITEM   OK=EDIT   B1=SALIR");

  u8g2.sendBuffer();
}

} // namespace lcd_ui
