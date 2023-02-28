#include "timer.hpp"

Timer::Timer(CPU& cpu, MMU& mmu, Interrupt& interrupt)
    : cpu_{cpu}, mmu_{mmu}, interrupt_{interrupt}, div_counter_{0}, tima_counter_{0} {
  mmu_.WriteByte(0xFF04, 0xAB); // DIV
  mmu_.WriteByte(0xFF05, 0x00); // TIMA
  mmu_.WriteByte(0xFF06, 0x00); // TMA
  mmu_.WriteByte(0xFF07, 0xF8); // TAC
}

void Timer::TickTimer() {
  int total_cycles = cpu_.tcycles;
  IncrementDIV(total_cycles);
  TickTIMA(total_cycles);
}

void Timer::IncrementDIV(int total_cycles) {
  div_counter_ += total_cycles;
  while (div_counter_ >= 256) {
    // DIV is incremented every 256 clock cycles (16384 Hz)
    mmu_.WriteByte(0xFF04, mmu_.ReadByte(0xFF04) + 1);
    div_counter_ -= 256;
  }
  // TODO: when stop instruction is executed, DIV will be reset
}

void Timer::TickTIMA(int total_cycles) {
  uint8_t tac = mmu_.ReadByte(0xFF07);

  // if timer enable bit (only for TIMA) is on
  if (tac & 0b100) {
    int tima_inc_cycle;

    switch (tac & 0b11) {
      case 0b00:  // 4096 Hz
        tima_inc_cycle = 1024;
        break;
      case 0b01:  // 262144 Hz
        tima_inc_cycle = 16;
        break;
      case 0b10:  // 65536 Hz
        tima_inc_cycle = 64;
        break;
      case 0b11:  // 16384 Hz
        tima_inc_cycle = 256;
        break;
    }

    tima_counter_ += total_cycles;
    while (tima_counter_ >= tima_inc_cycle) {
      uint8_t tima = mmu_.ReadByte(0xFF05);
      if (tima == 0xFF) {  // TIMA overflow check
        interrupt_.SetIF(INTERRUPT_TIMER);
        mmu_.WriteByte(0xFF05, mmu_.ReadByte(0xFF06));  // TIMA is reset to TMA value
      } else {
        mmu_.WriteByte(0xFF05, tima + 1);
      }
      tima_counter_ -= tima_inc_cycle;
    }
  }
}