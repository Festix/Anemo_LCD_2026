#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

#include "config.h"
#include "wind_packet.h"
#include "crc16_modbus.h"
#include "lcd_ui.h"

// ===================== ESP-NOW RX =====================
static WindPacket lastPkt{};
static bool havePkt = false;

static uint32_t lastRxMs = 0;
static uint32_t lastSeq = 0;
static bool seqInit = false;

static uint32_t cntLost = 0;
static uint32_t cntBadCrc = 0;
static uint32_t cntBadMagic = 0;
static uint32_t cntBadLen = 0;

static bool validatePacket(const WindPacket& p) {
  if (p.magic != WIND_MAGIC || p.version != WIND_VER) return false;
  const uint8_t* bytes = (const uint8_t*)&p;
  uint16_t calc = crc16_modbus(bytes, sizeof(WindPacket) - sizeof(p.crc16));
  return (calc == p.crc16);
}

// Callback API vieja (Arduino-ESP32 v1/v2)
void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;

  if (len != (int)sizeof(WindPacket)) { cntBadLen++; return; }

  WindPacket p;
  memcpy(&p, data, sizeof(WindPacket));

  if (p.magic != WIND_MAGIC || p.version != WIND_VER) { cntBadMagic++; return; }
  if (!validatePacket(p)) { cntBadCrc++; return; }

  if (!seqInit) {
    seqInit = true;
  } else {
    uint32_t expected = lastSeq + 1;
    if (p.seq != expected) {
      if (p.seq > expected) cntLost += (p.seq - expected);
      // si p.seq < expected => reset del emisor, no contamos
    }
  }
  lastSeq = p.seq;

  lastPkt = p;
  havePkt = true;
  lastRxMs = millis();
}

// ===================== SETTINGS (Preferences) =====================
Preferences prefs;

struct Settings {
  int16_t dir_offset_deg = 0;   // -180..180
  float   speed_factor   = 1.0; // multiplicador
  uint8_t speed_src      = 0;   // 0=PPS, 1=RPM
};

static Settings cfg;

static void loadSettings() {
  prefs.begin("anemo", true);
  cfg.dir_offset_deg = prefs.getShort("dir_off", 0);
  cfg.speed_factor   = prefs.getFloat("spd_fac", 1.0f);
  cfg.speed_src      = prefs.getUChar("spd_src", 0);
  prefs.end();

  if (cfg.dir_offset_deg < -180) cfg.dir_offset_deg = -180;
  if (cfg.dir_offset_deg >  180) cfg.dir_offset_deg =  180;
  if (!(cfg.speed_factor > 0.0001f && cfg.speed_factor < 1000.0f)) cfg.speed_factor = 1.0f;
  if (cfg.speed_src > 1) cfg.speed_src = 0;
}

static void saveSettings() {
  prefs.begin("anemo", false);
  prefs.putShort("dir_off", cfg.dir_offset_deg);
  prefs.putFloat("spd_fac", cfg.speed_factor);
  prefs.putUChar("spd_src", cfg.speed_src);
  prefs.end();
}

static float applyDir(float dirDeg) {
  float d = dirDeg + (float)cfg.dir_offset_deg;
  while (d < 0) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

static float calcSpeed(const WindPacket& p) {
  float base = (cfg.speed_src == 0) ? (p.pps_centi / 100.0f)
                                    : (p.rpm_centi / 100.0f);
  return base * cfg.speed_factor;
}

// ===================== BOTONES (4) =====================
// Pullup interno. Activo LOW.
struct Btn {
  uint8_t pin = 0;
  bool stable = true;      // HIGH = suelto
  bool lastStable = true;
  uint32_t tDeb = 0;
};

static Btn b1, b2, b3, b4;

static uint8_t btnPressMask = 0; // flancos (press)
static uint8_t btnDownMask  = 0; // estado estable (down=1)

static void buttonsBegin() {
  b1.pin = BTN1_PIN; b2.pin = BTN2_PIN; b3.pin = BTN3_PIN; b4.pin = BTN4_PIN;

  Btn* bs[] = {&b1,&b2,&b3,&b4};
  for (auto* b : bs) {
    pinMode(b->pin, INPUT_PULLDOWN);
    b->stable = digitalRead(b->pin);
    b->lastStable = b->stable;
    b->tDeb = millis();
  }
}

static void buttonsPoll() {
  const uint32_t now = millis();
  const uint32_t DB = 60;

  btnPressMask = 0;

  Btn* bs[] = {&b1,&b2,&b3,&b4};
  for (int i=0;i<4;i++) {
    auto* b = bs[i];
    bool raw = digitalRead(b->pin);

    if (raw != b->stable && (now - b->tDeb) >= DB) {
      b->lastStable = b->stable;
      b->stable = raw;
      b->tDeb = now;

      // flanco LOW->HIGH
      if (b->lastStable == false && b->stable == true) {
        btnPressMask |= (1u<<i);
      }
    }
  }

  // downMask: HIGH = presionado
  btnDownMask =
    ((b1.stable == true) ? 1u : 0u) |
    ((b2.stable == true) ? 2u : 0u) |
    ((b3.stable == true) ? 4u : 0u) |
    ((b4.stable == true) ? 8u : 0u);
}

static inline bool press(int i) { return (btnPressMask & (1u<<i)) != 0; }
static inline bool down (int i) { return (btnDownMask  & (1u<<i)) != 0; }
// i: 0=B1, 1=B2, 2=B3, 3=B4

// ===================== UI / PANTALLAS =====================
enum class Screen : uint8_t { MAIN, DIAG, INFO, CONFIG };

static Screen screen = Screen::MAIN;

// CONFIG (menu)
static lcd_ui::UiMode uiMode = lcd_ui::UiMode::MENU;
static int menuIndex = 0;
static const int MENU_COUNT = 3;

// Hold OK para entrar a CONFIG
static uint32_t okHoldStartMs = 0;
static bool okHoldArmed = false;

static void nextScreen() {
  if (screen == Screen::MAIN) screen = Screen::DIAG;
  else if (screen == Screen::DIAG) screen = Screen::INFO;
  else if (screen == Screen::INFO) screen = Screen::MAIN;
  else /*CONFIG*/ {}
}

static void prevScreen() {
  if (screen == Screen::MAIN) screen = Screen::INFO;
  else if (screen == Screen::INFO) screen = Screen::DIAG;
  else if (screen == Screen::DIAG) screen = Screen::MAIN;
  else /*CONFIG*/ {}
}

static void enterConfig() {
  screen = Screen::CONFIG;
  uiMode = lcd_ui::UiMode::MENU;
  // menuIndex se mantiene (o ponelo en 0 si querés)
}

static void exitConfigToMain() {
  screen = Screen::MAIN;
  uiMode = lcd_ui::UiMode::MENU;
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  loadSettings();
  buttonsBegin();
  lcd_ui::begin();

  WiFi.mode(WIFI_STA);
  Serial.print("RX MAC (STA): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: esp_now_init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);
  Serial.println("ESP-NOW RX ready");
}

void loop() {
  const uint32_t now = millis();
  buttonsPoll();

  // ===================== lógica HOLD OK (entrar a CONFIG) =====================
  if (screen != Screen::CONFIG) {
    if (down(3)) { // OK presionado
      if (!okHoldArmed) {
        okHoldArmed = true;
        okHoldStartMs = now;
      } else {
        if ((now - okHoldStartMs) >= MENU_HOLD_MS) {
          enterConfig();
          // bloquear para que no re-entre
          okHoldArmed = false;
        }
      }
    } else {
      okHoldArmed = false;
    }
  } else {
    okHoldArmed = false;
  }

  // ===================== navegación =====================
  if (screen != Screen::CONFIG) {
    // Entre pantallas: B1/B2
    if (press(1)) nextScreen(); // B2
    if (press(0)) prevScreen(); // B1
  } else {
    // CONFIG:
    // MENU: B2 next, B3 prev, OK edit, B1 exit
    // EDIT: B2 +, B3 -, OK save, B1 back
    if (uiMode == lcd_ui::UiMode::MENU) {
      if (press(0)) { exitConfigToMain(); } // salir de config
      if (press(1)) { menuIndex = (menuIndex + 1) % MENU_COUNT; } // next item
      if (press(2)) { menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT; } // prev item
      if (press(3)) { uiMode = lcd_ui::UiMode::EDIT; } // editar
    } else { // EDIT
      if (press(0)) { uiMode = lcd_ui::UiMode::MENU; } // volver sin guardar
      if (press(3)) { saveSettings(); uiMode = lcd_ui::UiMode::MENU; } // guardar

      if (menuIndex == 0) { // Offset proa
        if (press(1)) cfg.dir_offset_deg = min<int16_t>(180, cfg.dir_offset_deg + 1);
        if (press(2)) cfg.dir_offset_deg = max<int16_t>(-180, cfg.dir_offset_deg - 1);
      } else if (menuIndex == 1) { // Factor vel.
        if (press(1)) cfg.speed_factor *= 1.02f;
        if (press(2)) cfg.speed_factor /= 1.02f;
        if (cfg.speed_factor < 0.0001f) cfg.speed_factor = 0.0001f;
        if (cfg.speed_factor > 1000.0f) cfg.speed_factor = 1000.0f;
      } else if (menuIndex == 2) { // Fuente vel.
        if (press(1) || press(2)) cfg.speed_src = 1 - cfg.speed_src;
      }
    }
  }

  // ===================== Render (5 Hz) =====================
  static uint32_t lastDraw = 0;
  if (now - lastDraw >= LCD_FPS_MS) {
    lastDraw = now;

    bool ok = havePkt;
    uint32_t age = ok ? (now - lastRxMs) : 0;

    if (ok && age > NO_DATA_MS) ok = false;

    // Pre-cálculos
    float dirRaw = ok ? (lastPkt.angle_cdeg / 100.0f) : 0.0f;
    float dirCorr = applyDir(dirRaw);

    float pps = ok ? (lastPkt.pps_centi / 100.0f) : 0.0f;
    float rpm = ok ? (lastPkt.rpm_centi / 100.0f) : 0.0f;
    float spd = ok ? calcSpeed(lastPkt) : 0.0f;

    lcd_ui::SettingsView view{cfg.dir_offset_deg, cfg.speed_factor, cfg.speed_src};

    switch (screen) {
      case Screen::MAIN:
        lcd_ui::renderMain(ok ? &lastPkt : nullptr, ok, age, dirCorr, spd);
        break;

      case Screen::DIAG:
        lcd_ui::renderDiag(ok ? &lastPkt : nullptr, ok, age,
                           dirRaw, dirCorr,
                           pps, rpm, spd,
                           cfg.dir_offset_deg,
                           cntLost, cntBadLen, cntBadMagic, cntBadCrc);
        break;

      case Screen::INFO:
        lcd_ui::renderInfo(ok ? &lastPkt : nullptr, ok, age, view, dirCorr, spd);
        break;

      case Screen::CONFIG:
        lcd_ui::renderMenu(uiMode, menuIndex, view);
        break;
    }
  }

  // ===================== Serial debug (1 Hz) =====================
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    if (!havePkt) {
      Serial.printf("[NO PKT] badLen=%lu badMagic=%lu badCrc=%lu\n",
                    (unsigned long)cntBadLen,
                    (unsigned long)cntBadMagic,
                    (unsigned long)cntBadCrc);
    } else {
      uint32_t age = now - lastRxMs;
      float dirCorr = applyDir(lastPkt.angle_cdeg / 100.0f);
      float pps = lastPkt.pps_centi / 100.0f;
      float rpm = lastPkt.rpm_centi / 100.0f;
      float spd = calcSpeed(lastPkt);

      Serial.printf("[OK] seq=%lu age=%lums dir=%.2f pps=%.2f rpm=%.2f spd=%.2f vbat=%umV status=0x%04X lost=%lu badCrc=%lu badMagic=%lu\n",
                    (unsigned long)lastPkt.seq,
                    (unsigned long)age,
                    dirCorr, pps, rpm, spd,
                    lastPkt.vbat_mV,
                    lastPkt.status,
                    (unsigned long)cntLost,
                    (unsigned long)cntBadCrc,
                    (unsigned long)cntBadMagic);
    }
  }
}
