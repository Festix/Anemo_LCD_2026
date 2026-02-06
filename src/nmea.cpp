#include "nmea.h"
#include <string.h>
#include <math.h>

namespace nmea {

static Stream* s_io = nullptr;
static Config s_cfg;
static uint32_t s_last_out_ms = 0;

// ----------------- checksum -----------------
uint8_t checksumBody(const char* body) {
  // XOR de todo el "body" (sin '$' y sin '*')
  uint8_t cs = 0;
  for (const char* p = body; *p; ++p) cs ^= (uint8_t)(*p);
  return cs;
}

bool validateLine(const char* line) {
  // Linea tipo: $.....*HH
  if (!line || line[0] != '$') return false;
  const char* star = strchr(line, '*');
  if (!star || (star - line) < 2) return false;

  uint8_t cs = 0;
  for (const char* p = line + 1; p < star; ++p) cs ^= (uint8_t)(*p);

  unsigned got = 0;
  if (sscanf(star + 1, "%2x", &got) != 1) return false;
  return cs == (uint8_t)got;
}

static void printMWV(float dir_deg, float speed_kn, bool valid) {
  if (!s_io) return;

  // Normalizar dirección
  while (dir_deg < 0) dir_deg += 360.0f;
  while (dir_deg >= 360.0f) dir_deg -= 360.0f;

  // Sanear valores raros (NaN/Inf rompen NMEA)
  if (!isfinite(dir_deg)) dir_deg = 0;
  if (!isfinite(speed_kn) || speed_kn < 0) speed_kn = 0;

  char body[64];
  snprintf(body, sizeof(body),
           "%sMWV,%03d,R,%.1f,N,%c",
           s_cfg.talker ? s_cfg.talker : "WI",
           (int)lroundf(dir_deg),
           speed_kn,
           valid ? 'A' : 'V');

  uint8_t cs = checksumBody(body);

  char line[96];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);

  // 1) Enviar a NMEA (Serial2)
  s_io->print(line);

  // 2) Debug opcional por USB (para ver EXACTO qué se arma)
  //    Descomentá 1 minuto:
  // Serial.print("[NMEA OUT] ");
  // Serial.print(line);

  // 3) Auto-check del checksum (si falla, lo sabés al instante)
  // if (!validateLine(line)) {
  //   Serial.print("[NMEA OUT] INVALID LINE: ");
  //   Serial.print(line);
  // }
}


// ----------------- IN: lectura de lineas -----------------
static bool readLine(char* buf, size_t buflen) {
  static size_t n = 0;
  if (!s_io) return false;

  while (s_io->available()) {
    char c = (char)s_io->read();
    if (c == '\r') continue;

    if (c == '\n') {
      buf[n] = 0;
      n = 0;
      return true;
    }

    if (n + 1 < buflen) {
      buf[n++] = c;
    } else {
      // overflow -> descartar
      n = 0;
    }
  }
  return false;
}

static void handleLine(const char* line) {
  // Por ahora: solo valida y deja “hook” para comandos propietarios
  if (!validateLine(line)) return;

  // Ejemplo futuro de comando propietario:
  // $PANA,CH,1*hh   -> set channel
  // $PANA,OFF,-12*hh -> offset
  // $PANA,FAC,1.23*hh -> factor
  if (strncmp(line, "$PANA,", 6) == 0) {
    // acá después lo conectamos con tu config/prefs
    // por ahora solo log
    if (s_io) s_io->printf("[NMEA IN] %s\r\n", line);
  }
}

// ----------------- API pública -----------------
void begin(Stream& io, const Config& cfg) {
  s_io = &io;
  s_cfg = cfg;
  s_last_out_ms = millis();
}

void tickOut(float dir_deg, float speed_kn, bool valid) {
  if (!s_cfg.enabled_out || !s_io) return;

  const uint32_t now = millis();
  if ((now - s_last_out_ms) < s_cfg.out_period_ms) return;
  s_last_out_ms = now;

  printMWV(dir_deg, speed_kn, valid);
}

void pollIn() {
  if (!s_cfg.enabled_in || !s_io) return;

  char line[96];
  if (readLine(line, sizeof(line))) {
    handleLine(line);
  }
}

} // namespace nmea
