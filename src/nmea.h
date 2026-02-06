
#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace nmea {

struct Config {
  bool enabled_out = true;
  bool enabled_in  = false;   // por ahora apagado si querés
  uint32_t out_period_ms = 1000; // 1 Hz
  const char* talker = "WI";  // "WI" recomendado
};

// Inicializa el módulo con un Stream (Serial/Serial2/etc.)
void begin(Stream& io, const Config& cfg = Config());

// Enviar MWV periódicamente (OUT). Llamar desde loop().
void tickOut(float dir_deg, float speed_kn, bool valid);

// Leer y procesar NMEA entrante (IN). Llamar desde loop() si enabled_in=true.
void pollIn();

// Helpers por si los querés usar afuera
uint8_t checksumBody(const char* body);
bool validateLine(const char* line);

} // namespace nmea
