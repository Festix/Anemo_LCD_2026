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
    case 3: return "ESP-NOW Canal";

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

void renderMain(const WindPacket* p, bool ok, uint32_t /*age_ms*/,
                float dir_deg_corrected, float speed_value, float holdProgress)
{
  u8g2.clearBuffer();

  // --- Layout (128x64) ---
  const int cx = 31;
  const int cy = 32;
  const int r  = 31;
  const int xText = 76;

  bool validDir = ok && p && ((p->status & (1u << 1)) != 0);

  // ===== Rosa grande =====
  u8g2.drawCircle(cx, cy, r);
  u8g2.drawCircle(cx, cy, r - 1);

  // Marcas internas (N/E/S/O)
  const int tickOuter = r - 1;
  const int tickInner = r - 10;

  u8g2.drawLine(cx, cy - tickOuter, cx, cy - tickInner); // N
  u8g2.drawLine(cx, cy + tickOuter, cx, cy + tickInner); // S
  u8g2.drawLine(cx - tickOuter, cy, cx - tickInner, cy); // W
  u8g2.drawLine(cx + tickOuter, cy, cx + tickInner, cy); // E

  if (!validDir) {
    u8g2.drawLine(cx - 12, cy - 12, cx + 12, cy + 12);
    u8g2.drawLine(cx - 12, cy + 12, cx + 12, cy - 12);
  } else {
    // ===== Flecha: triángulo largo, relleno, angosto, base en el centro =====
    float a = deg2rad(dir_deg_corrected - 90.0f);

    // Punta casi en el borde
    const float tipLen = (float)(r - 1);
    int xt = cx + (int)(cosf(a) * tipLen);
    int yt = cy + (int)(sinf(a) * tipLen);

    // Base exactamente en el centro
    const float baseLen = 0.0f;
    int xb = cx + (int)(cosf(a) * baseLen);
    int yb = cy + (int)(sinf(a) * baseLen);

    // Perpendicular para ancho (más angosto)
    const float w = 2.5f; // angosto
    float ap = a + 1.57079632679f;

    int xL = xb + (int)(cosf(ap) * w);
    int yL = yb + (int)(sinf(ap) * w);
    int xR = xb - (int)(cosf(ap) * w);
    int yR = yb - (int)(sinf(ap) * w);

    u8g2.drawTriangle(xt, yt, xL, yL, xR, yR);

    // Centro prolijo
    u8g2.drawDisc(cx, cy, 2);
  }

  // ===== Textos a la derecha =====
  char b[32];

  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(xText, 14, "DIR");
  u8g2.drawStr(xText, 40, "SPD");

  u8g2.setFont(u8g2_font_7x13B_tf);
  if (ok && p) snprintf(b, sizeof(b), "%.1f%c", dir_deg_corrected, 176);
  else        snprintf(b, sizeof(b), "--.-%c", 176);
  u8g2.drawStr(xText, 28, b);

  if (ok && p) snprintf(b, sizeof(b), "%.2f", speed_value);
  else        snprintf(b, sizeof(b), "--.--");
  u8g2.drawStr(xText, 54, b);

  // ===== Pie: barra hold o estado =====
  u8g2.setFont(u8g2_font_5x8_tf);

  if (holdProgress >= 0.0f) {
    if (holdProgress > 1.0f) holdProgress = 1.0f;

    const int x = 0, y = 56, wBar = 128, hBar = 8;
    u8g2.drawFrame(x, y, wBar, hBar);

    int fill = (int)((wBar - 2) * holdProgress);
    if (fill < 0) fill = 0;
    if (fill > (wBar - 2)) fill = (wBar - 2);

    u8g2.drawBox(x + 1, y + 1, fill, hBar - 2);
  } else {
    if (!ok || !p) u8g2.drawStr(0, 63, "NOK");
    else           u8g2.drawStr(0, 63, "OK");
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
  u8g2.drawStr(0, 12, "Info - Diagnostico");
  u8g2.drawLine(0,15,128,15);
  u8g2.setFont(u8g2_font_5x8_tf);
  char b[32];

  // Línea 1: link
  if (!ok || !p) {
    u8g2.drawStr(0, 24, "LINK: OFFLINE");
  } else {
    u8g2.drawStr(0, 24, "LINK: ONLINE");
  }

  // Línea 2: SEQ
  snprintf(b, sizeof(b), "Sequence : %lu", (unsigned long)seq);
  u8g2.drawStr(0, 34, b);

  // Línea 3: AGE
  snprintf(b, sizeof(b), "Age: %lu ms", (unsigned long)age_ms);
  u8g2.drawStr(0, 44, b);

  // Línea 4: STATUS
  snprintf(b, sizeof(b), "Status: 0x%04X", (unsigned)status);
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

  // Marco
  u8g2.drawFrame(0, 0, 128, 64);

  // Título
  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(6, 14, "CONFIG");

  // Subtítulo modo
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(86, 14, (mode == UiMode::EDIT) ? "EDIT" : "MENU");

  // Item (label)
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(6, 32, menuLabel(menuIndex));

  // Caja de valor
  u8g2.drawFrame(6, 38, 116, 18);

  char v[32];
  if (menuIndex == 0) snprintf(v, sizeof(v), "%d%c", (int)cfg.dir_offset_deg, 176);
  else if (menuIndex == 1) snprintf(v, sizeof(v), "x%.3f", cfg.speed_factor);
  else if (menuIndex == 2) snprintf(v, sizeof(v), "%s", (cfg.speed_src==0) ? "PPS" : "RPM");
  else if (menuIndex == 3) snprintf(v, sizeof(v), "CH %u", (unsigned)cfg.espnow_channel);
  else snprintf(v, sizeof(v), "-");

  u8g2.setFont(u8g2_font_7x13B_tf);
  u8g2.drawStr(10, 52, v);

  // Footer: ayuda corta + MAC
  u8g2.setFont(u8g2_font_5x8_tf);

  // Footer: MAC solo en el item de Canal
  if (menuIndex == 3 && cfg.macStr && cfg.macStr[0]) {
    char m[24];
    snprintf(m, sizeof(m), "MAC %s", cfg.macStr);
    u8g2.drawStr(6, 63, m);
  }

  // Ayuda muy corta (arriba del MAC si querés, o alternar)
  if (mode == UiMode::EDIT) {
    u8g2.drawStr(6, 24, "B2:+  B3:-  OK:GUARDA");
  } else {
    u8g2.drawStr(6, 24, "B2/B3:ITEM  OK:EDIT");
  }

  u8g2.sendBuffer();
}


} // namespace lcd_ui
