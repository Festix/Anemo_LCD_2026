#pragma once
#include <Arduino.h>

// ===================== LCD ST7920 (U8g2 SW SPI) =====================
static constexpr uint8_t LCD_CLK = 27;
static constexpr uint8_t LCD_DAT = 25;
static constexpr uint8_t LCD_CS  = 12;
static constexpr uint8_t LCD_RST = 26;

// ===================== BOTONES =====================
// Asumo: botones a GND + INPUT_PULLUP (activo LOW)
static constexpr uint8_t BTN1_PIN = 19; // BACK
static constexpr uint8_t BTN2_PIN = 21; // NEXT / +
static constexpr uint8_t BTN3_PIN = 22; // PREV / -
static constexpr uint8_t BTN4_PIN = 23; // OK

// ===================== UI =====================
static constexpr uint32_t LCD_FPS_MS = 200;    // refresco 5 Hz
static constexpr uint32_t NO_DATA_MS = 2000;   // si no hay paquetes en 2s -> NO DATA

// Para evitar falsos toques: mantener OK apretado para entrar a Config
static constexpr uint32_t MENU_HOLD_MS = 1200; // 1.2s

// ===================== ESP-NOW =====================
static constexpr uint8_t ESPNOW_CHANNEL = 1;   // poné el mismo canal que el transmisor

// ===================== SERIAL2 config pins =====================
static constexpr uint8_t RX2_PIN = 16; // NMEA IN
static constexpr uint8_t TX2_PIN = 17; // NMEA OUT


