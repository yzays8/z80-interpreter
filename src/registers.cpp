#include "registers.hpp"

Registers::Registers()
    : a{0x01}, f{0xB0}, b{0x00}, c{0x13}, d{0x00}, e{0xD8}, h{0x01}, l{0x4D},
      af{a, f}, bc{b, c}, de{d, e}, hl{h, l}, pc{0x0100}, sp{0xFFFE} {
}

void Registers::SetZeroFlag(int res, bool reset) {  // z
  if (reset || (res != 0)) {
    f &= 0b01111111; // z = 0
  } else {
    f |= 0b10000000; // z = 1
  }
}

void Registers::SetSubtractionFlag(bool flag) { // n
  if (flag) {
    f |= 0b01000000; // n = 1
  } else {
    f &= 0b10111111; // n = 0
  }
}

void Registers::SetHalfCarryFlag(bool flag) { // h
  if (flag) {
    f |= 0b00100000; // h = 1
  } else {
    f &= 0b11011111; // h = 0
  }
}

void Registers::SetCarryFlag(bool flag) { // c
  if (flag) {
    f |= 0b00010000; // c = 1
  } else {
    f &= 0b11101111; // c = 0
  }
}

bool Registers::GetZeroFlag() {
  return (f >> 7) & 0x1;
}

bool Registers::GetSubtractionFrag() {
  return (f >> 6) & 0x1;
}

bool Registers::GetHalfCarryFlag() {
  return (f >> 5) & 0x1;
}

bool Registers::GetCarryFlag() {
  return (f >> 4) & 0x1;
}