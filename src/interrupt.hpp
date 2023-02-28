#pragma once

#include <array>

#include "mmu.hpp"

enum InterruptType{
  INTERRUPT_VBLANK,
  INTERRUPT_LCD,
  INTERRUPT_TIMER,
  INTERRUPT_SERIAL_LINK,
  INTERRUPT_JOYPAD,
  NO_INTERRUPT
};

const std::array<uint16_t, 5> kInterruptHandler = {0x0040, 0x0048, 0x0050, 0x0058, 0x0060};

class Interrupt {
 public:
  Interrupt(MMU& mmu);

  void SetIME(bool ime);
  void SetIE(InterruptType type);
  void SetIF(InterruptType type); // request interrupt
  void ResetIF(int type);
  bool GetIME();
  uint8_t GetIE();
  uint8_t GetIF();

  int CheckInterrupt();

 private:
  bool ime_;
  MMU& mmu_;
};