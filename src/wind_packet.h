#pragma once
#include <stdint.h>

struct __attribute__((packed)) WindPacket {
  uint16_t magic;          // 0x574E = 'WN'
  uint16_t version;        // 1
  uint32_t seq;
  uint32_t timestamp_ms;
  uint16_t raw_angle;      // 0..4095
  uint16_t angle_cdeg;     // 0..35999 => deg = /100
  uint16_t pps_centi;      // pps = /100
  uint16_t rpm_centi;      // rpm = /100
  uint16_t vbat_mV;        // 0 si no usado
  uint16_t status;         // flags
  uint16_t i2c_err_count;
  uint16_t crc16;          // CRC16-Modbus de todo menos este campo
};

static_assert(sizeof(WindPacket) == 28, "WindPacket must be 28 bytes");
static constexpr uint16_t WIND_MAGIC = 0x574E;
static constexpr uint16_t WIND_VER   = 1;
