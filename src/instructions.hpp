#pragma once

#include "registers.hpp"
#include "mmu.hpp"

class Instructions {
 public:
  Instructions(Registers& registers, MMU& mmu);

  int GetBranchCycle(); // return the number of additional cycles in an instruction with a branch

  void Load(RegisterU16& op1);
  void Load(uint8_t& op1);
  void Load(uint8_t& op1, uint8_t& op2);
  void Load(uint8_t& op1, RegisterU16& op2);
  void Load(RegisterU16& op1, uint8_t& op2);

  void Ret(bool flag);
  void Pop(RegisterU16& op1);
  void Push(RegisterU16& op1);
  void Jp(bool flag);
  void Jr(bool flag);
  void Call(bool flag);
  void Rst(uint16_t addr);

  void Add(RegisterU16& op1, RegisterU16& op2);
  void Add(uint8_t& op1, uint8_t& op2);
  void Adc(uint8_t& op1, uint8_t& op2);
  void Sub(uint8_t& op1);
  void Sbc(uint8_t& op1, uint8_t& op2);
  void Inc(RegisterU16& op1);
  void Inc(uint8_t& op1);
  void Dec(RegisterU16& op1);
  void Dec(uint8_t& op1);

  void And(uint8_t& op1);
  void Or(uint8_t& op1);
  void Xor(uint8_t& op1);
  void Cp(uint8_t& op1);

  // CB prefixed instructions below

  void Rlc(uint8_t& op1); // rotate left
  void Rrc(uint8_t& op1); // rotate right
  void Rl(uint8_t& op1);  // rotate left through carry
  void Rr(uint8_t& op1);  // rotate right through carry

  void Sla(uint8_t& op1);   // shift left arithmetic
  void Sra(uint8_t& op1);   // shift right arithmetic
  void Swap(uint8_t& op1);  // exchange low/hi-nibble
  void Srl(uint8_t& op1);   // shift right logical

  void Bit(int op1, uint8_t& op2);  // test bit op1 of op2
  void Res(int op1, uint8_t& op2);  // reset bit op1 of op2
  void Set(int op1, uint8_t& op2);  // set bit op1 of op2

 private:
  Registers& registers_;
  MMU& mmu_;
  int branch_cycles_;
};