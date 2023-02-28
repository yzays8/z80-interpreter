#pragma once

#include <cstdint>

struct RegisterU16 {
  uint8_t& hi;
  uint8_t& lo;

  uint16_t Get() {
    return static_cast<uint16_t>((hi << 8) | lo);
  }
  void Set(uint16_t val) {
    lo = static_cast<uint8_t>(val & 0xFF);
    hi = static_cast<uint8_t>((val >> 8) & 0xFF);
  }
};

class Registers {
 public:
  Registers();

  void SetZeroFlag(int res, bool reset);  // z
  void SetSubtractionFlag(bool flag);     // n
  void SetHalfCarryFlag(bool flag);       // h
  void SetCarryFlag(bool flag);           // c

  bool GetZeroFlag();
  bool GetSubtractionFrag();  // for DAA instruction
  bool GetHalfCarryFlag();    // for DAA instruction
  bool GetCarryFlag();

  uint8_t a, f, b, c, d, e, h, l;
  RegisterU16 af, bc, de, hl;
  uint16_t pc, sp;
};