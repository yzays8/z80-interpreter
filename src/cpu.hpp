#pragma once

#include <cstdint>
#include <array>

#include "registers.hpp"
#include "instructions.hpp"
#include "mmu.hpp"
#include "interrupt.hpp"

// instructions with branches are not considered here
const std::array<int, 0x100> tcycles_table_ = {
  4, 12, 8, 8, 4, 4, 8, 4, 20, 8, 8, 8, 4, 4, 8, 4,       // 0x00 - 0x0F
  4, 12, 8, 8, 4, 4, 8, 4, 12, 8, 8, 8, 4, 4, 8, 4,       // 0x10 - 0x1F
  0, 12, 8, 8, 4, 4, 8, 4, 0, 8, 8, 8, 4, 4, 8, 4,        // 0x20 - 0x2F
  0, 12, 8, 8, 12, 12, 12, 4, 0, 8, 8, 8, 4, 4, 8, 4,     // 0x30 - 0x3F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x40 - 0x4F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x50 - 0x5F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x60 - 0x6F
  8, 8, 8, 8, 8, 8, 4, 8, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x70 - 0x7F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x80 - 0x8F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0x90 - 0x9F
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0xA0 - 0xAF
  4, 4, 4, 4, 4, 4, 8, 4, 4, 4, 4, 4, 4, 4, 8, 4,         // 0xB0 - 0xBF
  0, 12, 0, 16, 0, 16, 8, 16, 0, 16, 0, 4, 0, 24, 8, 16,  // 0xC0 - 0xCF
  0, 12, 0, 0, 0, 16, 8, 16, 0, 16, 0, 0, 0, 0, 8, 16,    // 0xD0 - 0xDF
  12, 12, 8, 0, 0, 16, 8, 16, 16, 4, 16, 0, 0, 0, 8, 16,  // 0xE0 - 0xEF
  12, 12, 8, 4, 0, 16, 8, 16, 12, 8, 16, 4, 0, 0, 8, 16   // 0xF0 - 0xFF
};

const std::array<int, 0x100> tcycles_table_cb_prefixed_ = {
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x00 - 0x0F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x10 - 0x1F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x20 - 0x2F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x30 - 0x3F
  8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x40 - 0x4F
  8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x50 - 0x5F
  8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x60 - 0x6F
  8, 8, 8, 8, 8, 8, 12, 8, 8, 8, 8, 8, 8, 8, 12, 8, // 0x70 - 0x7F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x80 - 0x8F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0x90 - 0x9F
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xA0 - 0xAF
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xB0 - 0xBF
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xC0 - 0xCF
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xD0 - 0xDF
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, // 0xE0 - 0xEF
  8, 8, 8, 8, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8  // 0xF0 - 0xFF
};

class CPU {
 public:
  CPU(Registers& registers, Instructions& instructions, MMU& mmu, Interrupt& interrupt);

  void InterpretInstruction(const uint8_t opcode);
  void InterpretInstructionEx(const uint8_t opcode);  // for CB prefixed instructions
  void DebugInstruction(const uint8_t opcode);
  void TickCPU();

  int tcycles;  // CPU clock cycle count, which is reset every CPU run-loop

 private:
  Registers& registers_;
  Instructions& instructions_;
  MMU& mmu_;
  Interrupt& interrupt_;

  bool halt_;
};