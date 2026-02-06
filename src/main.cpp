#include <Arduino.h>
#include <WiFi.h>
#include <esp_WiFi.h>
#include <esp_now.h>
#include <Preferences.h>

#include "config.h"
#include "lcd_ui.h"
#include "wind_packet.h"
#include "nmea.h"

// ===================== Settings persistentes =====================
struct AppConfig {
  int16_t dir_offset_deg = 0;   // -180..180
  float   speed_factor  = 1.0f; // multiplicador
  uint8_t speed_src     = 0;    // 0=PPS, 1=RPM
  uint8_t espnow_channel = 1;  // 1..13
};

static char macStr[18] = {0}; // "AA:BB:CC:DD:EE:FF"

static Preferences prefs;
static AppConfig cfg;

static void loadSettings() {
  prefs.begin("anemo", true);
  cfg.dir_offset_deg = prefs.getShort("dir_off", 0);
  cfg.speed_factor  = prefs.getFloat("spd_fac", 1.0f);
  cfg.speed_src     = prefs.getUChar("spd_src", 0);
  cfg.espnow_channel = prefs.getUChar("esp_ch", 1);
  if (cfg.espnow_channel < 1) cfg.espnow_channel = 1;
  if (cfg.espnow_channel > 13) cfg.espnow_channel = 13;
  prefs.end();

  cfg.dir_offset_deg = constrain(cfg.dir_offset_deg, -180, 180);
  if (!(cfg.speed_factor > 0.0001f && cfg.speed_factor < 1000.0f)) cfg.speed_factor = 1.0f;
  cfg.speed_src = (cfg.speed_src > 1) ? 0 : cfg.speed_src;
}

static void saveSettings() {
  prefs.begin("anemo", false);
  prefs.putShort("dir_off", cfg.dir_offset_deg);
  prefs.putFloat("spd_fac", cfg.speed_factor);
  prefs.putUChar("spd_src", cfg.speed_src);
  prefs.putUChar("esp_ch", cfg.espnow_channel);
  prefs.end();
}

// ===================== CRC16 Modbus (para validar paquete) =====================
static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

// ===================== Estado ESPNOW =====================
static volatile uint32_t rxCount = 0;
static volatile bool havePkt = false;
static WindPacket lastPkt {};
static uint32_t lastRxMs = 0;

static uint32_t lastSeq = 0;
static bool haveSeq = false;

static uint32_t cntLost = 0;
static uint32_t cntBadLen = 0;
static uint32_t cntBadMagic = 0;
static uint32_t cntBadCrc = 0;

// ===================== Botones touch =====================
struct Btn {
  uint8_t pin = 0;
  bool stable = false;      // estado estable
  bool lastStable = false;  // estado estable anterior
  uint32_t tDeb = 0;
};

static Btn b[4];
static uint8_t btnPressMask = 0; // flancos LOW->HIGH
static uint8_t btnDownMask  = 0; // estado estable (down=1)

static inline bool press(uint8_t i) { return (btnPressMask & (1u << i)) != 0; }
static inline bool down(uint8_t i)  { return (btnDownMask  & (1u << i)) != 0; }

static void buttonsBegin() {
  const uint8_t pins[4] = { BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN };
  for (int i = 0; i < 4; i++) {
    b[i].pin = pins[i];
    pinMode(b[i].pin, INPUT_PULLDOWN);
    b[i].stable = digitalRead(b[i].pin);
    b[i].lastStable = b[i].stable;
    b[i].tDeb = millis();
  }
}

static void buttonsPoll() {
  const uint32_t now = millis();
  const uint32_t DB = 60;

  btnPressMask = 0;
  btnDownMask = 0;

  for (int i = 0; i < 4; i++) {
    bool raw = digitalRead(b[i].pin);

    if (raw != b[i].stable) {
      if ((now - b[i].tDeb) >= DB) {
        b[i].lastStable = b[i].stable;
        b[i].stable = raw;
        b[i].tDeb = now;

        // flanco LOW->HIGH
        if (b[i].lastStable == false && b[i].stable == true) {
          btnPressMask |= (1u << i);
        }
      }
    } else {
      b[i].tDeb = now; // estable: resetea ventana
    }

    if (b[i].stable) btnDownMask |= (1u << i);
  }
}

// ===================== UI: pantallas y menú =====================
enum class Screen : uint8_t { MAIN, DIAG };
static Screen screen = Screen::MAIN;

static bool inConfig = false;
static lcd_ui::UiMode uiMode = lcd_ui::UiMode::MENU;
static int menuIndex = 0;
static constexpr int MENU_COUNT = 4;

// Hold OK para entrar a CONFIG
static bool okHoldArmed = false;
static uint32_t okHoldStartMs = 0;

static void enterConfig() {
  inConfig = true;
  uiMode = lcd_ui::UiMode::MENU;
  menuIndex = 0;
}

static void exitConfigToMain() {
  inConfig = false;
  screen = Screen::MAIN;
}

static void toggleScreen() {
  screen = (screen == Screen::MAIN) ? Screen::DIAG : Screen::MAIN;
}

// ===================== ESPNOW callback =====================
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;

  
  rxCount++;

  if (len != (int)sizeof(WindPacket)) {
    cntBadLen++;
    return;
  }

  WindPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  if (pkt.magic != WIND_MAGIC || pkt.version != WIND_VER) {
    cntBadMagic++;
    return;
  }

  // CRC de todo menos el campo crc16
  const uint16_t calc = crc16_modbus((const uint8_t*)&pkt, sizeof(WindPacket) - sizeof(pkt.crc16));
  if (calc != pkt.crc16) {
    cntBadCrc++;
    return;
  }

  // lost por seq
  if (haveSeq) {
    const uint32_t expect = lastSeq + 1;
    if (pkt.seq != expect) {
      if (pkt.seq > expect) cntLost += (pkt.seq - expect);
      else cntLost++; // wrap o reorder raro, cuenta 1
    }
  }
  lastSeq = pkt.seq;
  haveSeq = true;

  lastPkt = pkt;
  lastRxMs = millis();
  havePkt = true;
}


static void printChannel(const char* tag) {
  uint8_t ch; wifi_second_chan_t sch;
  esp_err_t e = esp_wifi_get_channel(&ch, &sch);
  Serial.printf("[%s] get_channel err=%d ch=%u\n", tag, (int)e, ch);
}

static void forceChannel(uint8_t ch) {
  WiFi.mode(WIFI_STA);
  delay(150);

  // Asegurar que el driver WiFi esté iniciado
  esp_err_t s = esp_wifi_start();
  Serial.printf("[WiFi] start=%d\n", (int)s);

  // (opcional) apagar power save
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Set channel
  esp_err_t e1 = esp_wifi_set_promiscuous(true);
  esp_err_t e2 = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_err_t e3 = esp_wifi_set_promiscuous(false);

  Serial.printf("[CH] prom_on=%d set_ch=%d prom_off=%d (want %u)\n",
                (int)e1, (int)e2, (int)e3, ch);

  uint8_t nowCh; wifi_second_chan_t sch;
  esp_err_t g = esp_wifi_get_channel(&nowCh, &sch);
  Serial.printf("[CH] get_channel=%d ch=%u\n", (int)g, nowCh);
}

static void espnowBegin() {
  Serial.printf("[WiFi] STA MAC=%s\n", WiFi.macAddress().c_str());
  printChannel("BEFORE");

  forceChannel(1);   // <-- ACA queda canal 1 sí o sí (o te muestra el error)

  esp_err_t e = esp_now_init();
  Serial.printf("[ESP-NOW] init=%d\n", (int)e);
  if (e == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    Serial.println("[ESP-NOW] recv_cb registered OK");
  }
}



// ===================== Setup/Loop =====================
void setup() {
 
  Serial.begin(115200);
 
  // Elegí el baud NMEA
  // - NMEA 0183 clásico: 4800
  // - NMEA “rápido” (AIS): 38400
  // - Si es tu propio enlace TTL: podés usar 9600/115200, pero para compatibilidad NMEA: 4800
  Serial2.begin(4800, SERIAL_8N1, RX2_PIN, TX2_PIN);
  delay(200);


  loadSettings();
  buttonsBegin();
  lcd_ui::begin();

  // WiFi/Channel/ESPNOW
  Serial.printf("[WiFi] STA MAC=%s\n", WiFi.macAddress().c_str());
  forceChannel(cfg.espnow_channel);


  esp_err_t e = esp_now_init();
  Serial.printf("[ESP-NOW] init=%d\n", (int)e);
  if (e == ESP_OK) {
    esp_now_register_recv_cb(onRecv);
    Serial.println("[ESP-NOW] recv_cb registered OK");
  }

  String mac = WiFi.macAddress();
  snprintf(macStr, sizeof(macStr), "%s", mac.c_str());

  nmea::Config nc;
  nc.enabled_out = true;
  nc.enabled_in  = false;   // lo activamos cuando quieras
  nc.out_period_ms = 1000;  // 1 Hz
  nc.talker = "WI";
  nmea::begin(Serial2, nc);

}

void loop() {
  const uint32_t now = millis();
  static uint32_t lastLogMs = 0;
  static uint32_t lastRxCount = 0;
  static bool lastOk = false;
  static float lastDirCorrDeg = 0.0f;
  static float lastSpdKn      = 0.0f;
  static bool  lastOkForNmea  = false;
  
  buttonsPoll();

  // ---- Estado datos ----
  bool ok = havePkt;
  uint32_t age = ok ? (now - lastRxMs) : 0;
  if (!ok || age > NO_DATA_MS) ok = false;

  // ---- OK hold para entrar config (con lockout hasta soltar) ----
  if (!inConfig) {
    if (down(3)) { // B4
      if (!okHoldArmed) {
        okHoldArmed = true;
        okHoldStartMs = now;
      } else {
        if ((now - okHoldStartMs) >= MENU_HOLD_MS) {
          enterConfig();
          okHoldArmed = false; // lockout: no re-entra hasta soltar
        }
      }
    } else {
      okHoldArmed = false;
    }
  } else {
    okHoldArmed = false;
  }

  // ---- Navegación ----
  if (!inConfig) {
    if (press(0) || press(1)) { // B1 o B2 alterna MAIN/DIAG
      toggleScreen();
    }
  } else {
    // CONFIG: MENU o EDIT
    if (uiMode == lcd_ui::UiMode::MENU) {
      if (press(0)) { // B1 salir
        exitConfigToMain();
      }
      if (press(1)) { // B2 next item
        menuIndex = (menuIndex + 1) % MENU_COUNT;
      }
      if (press(2)) { // B3 prev item
        menuIndex = (menuIndex + MENU_COUNT - 1) % MENU_COUNT;
      }
      if (press(3)) { // OK -> editar
        uiMode = lcd_ui::UiMode::EDIT;
      }
    } else { // EDIT
      if (press(0)) { // B1 volver sin guardar
        uiMode = lcd_ui::UiMode::MENU;
      }
      if (press(3)) { // OK guardar y volver
        saveSettings();
        if (menuIndex == 3) {
          // Reaplicar canal en vivo
          esp_now_deinit();
          forceChannel(cfg.espnow_channel);

          esp_err_t e = esp_now_init();
          Serial.printf("[ESP-NOW] reinit=%d ch=%u\n", (int)e, cfg.espnow_channel);
          if (e == ESP_OK) {
            esp_now_register_recv_cb(onRecv);
            Serial.println("[ESP-NOW] recv_cb registered OK");
          }
        }
        uiMode = lcd_ui::UiMode::MENU;
      }

      if (menuIndex == 0) { // Offset proa
        if (press(1)) cfg.dir_offset_deg = min<int16_t>(180, cfg.dir_offset_deg + 1);
        if (press(2)) cfg.dir_offset_deg = max<int16_t>(-180, cfg.dir_offset_deg - 1);
      } else if (menuIndex == 1) { // Factor vel.
        if (press(1)) cfg.speed_factor += 0.01f;
        if (press(2)) cfg.speed_factor = max(0.01f, cfg.speed_factor - 0.01f);
      } else if (menuIndex == 2) { // Fuente vel.
        if (press(1) || press(2)) cfg.speed_src = (cfg.speed_src == 0) ? 1 : 0;
      }
      else if (menuIndex == 3) { // ESP-NOW Channel
        if (press(1)) cfg.espnow_channel = min<uint8_t>(13, cfg.espnow_channel + 1);
        if (press(2)) cfg.espnow_channel = max<uint8_t>(1,  cfg.espnow_channel - 1);
      }
    }
  }

  // ---- Render (5 Hz) ----
  static uint32_t lastUiMs = 0;
  if ((now - lastUiMs) >= LCD_FPS_MS) {
    lastUiMs = now;

    // Cálculos
    const WindPacket* p = (ok) ? &lastPkt : nullptr;
   
    lastOkForNmea = ok; // default, se setea true si el paquete es OK y se procesa para NMEA

    float dirRawDeg = 0.0f;
    float dirCorrDeg = 0.0f;
    float pps = 0.0f;
    float rpm = 0.0f;
    float spd = 0.0f;

    if (p) {
      dirRawDeg  = (float)p->raw_angle * 360.0f / 4096.0f;
      dirCorrDeg = (float)p->angle_cdeg / 100.0f;

      // aplica offset (convención simple)
      dirCorrDeg += (float)cfg.dir_offset_deg;
      while (dirCorrDeg < 0)   dirCorrDeg += 360.0f;
      while (dirCorrDeg >= 360.0f) dirCorrDeg -= 360.0f;

      pps = (float)p->pps_centi / 100.0f;
      rpm = (float)p->rpm_centi / 100.0f;

      float base = (cfg.speed_src == 0) ? pps : rpm;
      spd = base * cfg.speed_factor;
      
      if (ok && p) {
        lastDirCorrDeg = dirCorrDeg;
        lastSpdKn      = spd;
      }
    }



    // hold progress (solo MAIN, solo mientras está armado)
    float holdProgress = -1.0f;
    if (!inConfig && screen == Screen::MAIN && okHoldArmed && down(3)) {
      holdProgress = (float)(now - okHoldStartMs) / (float)MENU_HOLD_MS;
      if (holdProgress < 0.0f) holdProgress = 0.0f;
      if (holdProgress > 1.0f) holdProgress = 1.0f;
    }

    // vista cfg para UI
    lcd_ui::SettingsView viewCfg;
    viewCfg.dir_offset_deg = cfg.dir_offset_deg;
    viewCfg.speed_factor   = cfg.speed_factor;
    viewCfg.speed_src      = cfg.speed_src;
    viewCfg.espnow_channel = cfg.espnow_channel;
    viewCfg.macStr         = macStr;


    if (inConfig) {
      lcd_ui::renderMenu(uiMode, menuIndex, viewCfg);
    } else if (screen == Screen::MAIN) {
      lcd_ui::renderMain(p, ok, age, dirCorrDeg, spd, holdProgress);
    } else {
      uint32_t seq = (ok && p) ? p->seq : 0;
      uint16_t st  = (ok && p) ? p->status : 0;
      lcd_ui::renderDiag(p, ok, age, seq, st, macStr, cntBadLen, cntBadMagic, cntBadCrc);
    }

  }

  if (millis() - lastLogMs >= 1000) {
    lastLogMs = millis();

    uint32_t c = rxCount; // lectura “rápida”
    uint32_t d = c - lastRxCount;
    lastRxCount = c;

    bool okNow = havePkt && ((millis() - lastRxMs) <= NO_DATA_MS);

    Serial.printf("[ESPNOW] +%lu pkt/s  ok=%d  age=%lums  seq=%lu  lost=%lu  badCrc=%lu badLen=%lu badMagic=%lu\n",
                  (unsigned long)d,
                  okNow ? 1 : 0,
                  okNow ? (unsigned long)(millis() - lastRxMs) : 0UL,
                  havePkt ? (unsigned long)lastPkt.seq : 0UL,
                  (unsigned long)cntLost,
                  (unsigned long)cntBadCrc,
                  (unsigned long)cntBadLen,
                  (unsigned long)cntBadMagic);

    if (okNow != lastOk) {
      Serial.printf("[LINK] %s\n", okNow ? "ONLINE" : "OFFLINE");
      lastOk = okNow;
    }

    nmea::tickOut(lastDirCorrDeg, lastSpdKn, lastOkForNmea);
  }
}
