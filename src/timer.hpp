#pragma once

#include "cpu.hpp"
#include "interrupt.hpp"

class Timer {
 public:
  Timer(CPU& cpu, MMU& mmu, Interrupt& interrupt);
  void TickTimer();

 private:
  CPU& cpu_;
  MMU& mmu_;
  Interrupt& interrupt_;
  int div_counter_, tima_counter_;

  void IncrementDIV(int total_cycles);
  void TickTIMA(int total_cycles);
};