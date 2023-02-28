#include "interrupt.hpp"
#include "mmu.hpp"

Interrupt::Interrupt(MMU& mmu) : ime_{0}, mmu_{mmu} {
  mmu_.WriteByte(0xFF0F, 0xE1);
}

void Interrupt::SetIME(bool ime) {
  ime_ = ime;
}

void Interrupt::SetIE(InterruptType type) {
  mmu_.WriteByte(0xFFFF, mmu_.ReadByte(0xFFFF) | (0x1 << type));
}

void Interrupt::SetIF(InterruptType type) {
  mmu_.WriteByte(0xFF0F, mmu_.ReadByte(0xFF0F) | (0x1 << type));
}

void Interrupt::ResetIF(int type) {
  mmu_.WriteByte(0xFF0F, mmu_.ReadByte(0xFF0F) ^ (0x1 << type));
}

bool Interrupt::GetIME() {
  return ime_;
}

uint8_t Interrupt::GetIE() {
  return mmu_.ReadByte(0xFFFF);
}

uint8_t Interrupt::GetIF() {
  return mmu_.ReadByte(0xFF0F);
}

int Interrupt::CheckInterrupt() {
  uint8_t if_register = GetIF();
  uint8_t ie_register = GetIE();
  int i;

  for (i = 0; i < 5; ++i) {
    if ((if_register & 0x1) && (ie_register & 0x1)) {
      break;
    }
    if_register >>= 1;
    ie_register >>= 1;
  }
  return i;
}