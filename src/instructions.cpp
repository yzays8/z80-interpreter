#include "instructions.hpp"
#include "registers.hpp"
#include "mmu.hpp"

Instructions::Instructions(Registers& registers, MMU& mmu)
    : registers_{registers}, mmu_{mmu}, branch_cycles_{0} {
}

int Instructions::GetBranchCycle() {
  int ret = branch_cycles_;
  branch_cycles_ = 0; // reset after call
  return ret;
}

void Instructions::Load(RegisterU16& op1) {
  op1.Set(mmu_.ReadShort(registers_.pc));
  registers_.pc += 2;
}

void Instructions::Load(uint8_t& op1) {
  op1 = mmu_.ReadByte(registers_.pc++);
}

void Instructions::Load(uint8_t& op1, uint8_t& op2) {
  op1 = op2;
}

void Instructions::Load(RegisterU16& op1, uint8_t& op2) {
  mmu_.WriteByte(op1.Get(), op2);
}

void Instructions::Load(uint8_t& op1, RegisterU16& op2) {
  op1 = mmu_.ReadByte(op2.Get());
}

void Instructions::Add(RegisterU16& op1, RegisterU16& op2) {
  registers_.SetHalfCarryFlag(((op1.Get() & 0xFFF) + (op2.Get() & 0xFFF)) > 0xFFF);
  registers_.SetSubtractionFlag(false);
  uint32_t res = op1.Get() + op2.Get();
  registers_.SetCarryFlag(res > 0xFFFF);
  op1.Set(static_cast<uint16_t>(res));
}

void Instructions::Add(uint8_t& op1, uint8_t& op2) {
  registers_.SetHalfCarryFlag(((op1 & 0xF) + (op2 & 0xF)) > 0xF);
  uint16_t res = op1 + op2;
  registers_.SetCarryFlag(res > 0xFF);
  op1 = static_cast<uint8_t>(res);
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
}

void Instructions::Adc(uint8_t& op1, uint8_t& op2) {
  registers_.SetHalfCarryFlag(((op1 & 0xF) + (op2 & 0xF) + registers_.GetCarryFlag()) > 0xF);
  uint16_t res = op1 + op2 + static_cast<uint8_t>(registers_.GetCarryFlag());
  registers_.SetCarryFlag(res > 0xFF);
  op1 = static_cast<uint8_t>(res);
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
}

void Instructions::Sub(uint8_t& op1) {
  registers_.SetSubtractionFlag(true);
  registers_.SetHalfCarryFlag((registers_.a & 0xF) < (op1 & 0xF));
  registers_.SetCarryFlag(registers_.a < op1);
  registers_.a -= op1;
  registers_.SetZeroFlag(registers_.a, false);
}

void Instructions::Sbc(uint8_t& op1, uint8_t& op2) {
  registers_.SetSubtractionFlag(true);
  registers_.SetHalfCarryFlag((op1 & 0xF) < ((op2 & 0xF) + registers_.GetCarryFlag()));
  bool old_cflag = registers_.GetCarryFlag();
  registers_.SetCarryFlag(op1 < (op2 + registers_.GetCarryFlag()));
  op1 -= (op2 + old_cflag);
  registers_.SetZeroFlag(op1, false);
}

void Instructions::Inc(RegisterU16& op1) {
  op1.Set(op1.Get() + 1);
}

void Instructions::Inc(uint8_t& op1) {
  registers_.SetHalfCarryFlag((op1 & 0xF) == 0xF);
  ++op1;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
}

void Instructions::Dec(RegisterU16& op1) {
  op1.Set(op1.Get() - 1);
}

void Instructions::Dec(uint8_t& op1) {
  registers_.SetHalfCarryFlag(!(op1 & 0xF));  // some documents are incorrect
  --op1;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(true);
}

void Instructions::Ret(bool flag) {
  if (flag) {
    registers_.pc = mmu_.ReadShort(registers_.sp);
    registers_.sp += 2; // shrink stack
    branch_cycles_ += 20;
  } else {
    branch_cycles_ += 8;
  }
}

void Instructions::Pop(RegisterU16& op1) {
  // only POP AF requires additional flag operations
  op1.Set(mmu_.ReadShort(registers_.sp));
  registers_.sp += 2;
}

void Instructions::Push(RegisterU16& op1) {
  registers_.sp -= 2; // extend stack
  mmu_.WriteShort(registers_.sp, op1.Get());
}

void Instructions::Jp(bool flag) {
  if (flag) {
    registers_.pc = mmu_.ReadShort(registers_.pc);
    branch_cycles_ += 16;
  } else {
    registers_.pc += 2;
    branch_cycles_ += 12;
  }
}

void Instructions::Jr(bool flag) {
  int8_t r8 = static_cast<int8_t>(mmu_.ReadByte(registers_.pc));
  ++registers_.pc;
  if (flag) {
    registers_.pc += r8;
    branch_cycles_ += 12;
  } else {
    branch_cycles_ += 8;
  }
}

void Instructions::Call(bool flag) {
  if (flag) {
    registers_.sp -= 2;  // extend stack
    uint16_t ret_addr = registers_.pc + 2;
    mmu_.WriteShort(registers_.sp, ret_addr);       // push future pc (return address) to stack
    registers_.pc = mmu_.ReadShort(registers_.pc);  // jump using immediate value got from current pc
    branch_cycles_ += 24;
  } else {
    registers_.pc += 2;
    branch_cycles_ += 12;
  }
}

void Instructions::Rst(uint16_t addr) {
  registers_.sp -= 2;
  mmu_.WriteShort(registers_.sp, registers_.pc);
  registers_.pc = addr;
}

void Instructions::And(uint8_t& op1) {
  registers_.a &= op1;
  registers_.SetZeroFlag(registers_.a, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(true);
  registers_.SetCarryFlag(false);
}

void Instructions::Or(uint8_t& op1) {
  registers_.a |= op1;
  registers_.SetZeroFlag(registers_.a, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(false);
}

void Instructions::Xor(uint8_t& op1) {
  registers_.a ^= op1;
  registers_.SetZeroFlag(registers_.a, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(false);
}

void Instructions::Cp(uint8_t& op1) {
  registers_.SetSubtractionFlag(true);
  registers_.SetHalfCarryFlag((registers_.a & 0xF) < (op1 & 0xF));
  registers_.SetCarryFlag(registers_.a < op1);
  registers_.SetZeroFlag(registers_.a - op1, false);
}


/* CB prefixed instructions below */


void Instructions::Rlc(uint8_t& op1) {
  bool msb = (op1 >> 7) & 0x1;
  op1 = (op1 << 1) | msb;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(msb);
}

void Instructions::Rrc(uint8_t& op1) {
  bool lsb = op1 & 0x1;
  op1 = (op1 >> 1) | (lsb << 7);
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(lsb);
}

void Instructions::Rl(uint8_t& op1) {
  bool msb = (op1 >> 7) & 0x1;
  op1 = (op1 << 1) | registers_.GetCarryFlag();
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(msb);
}

void Instructions::Rr(uint8_t& op1) {
  bool lsb = op1 & 0x1;
  op1 = (op1 >> 1) | (registers_.GetCarryFlag() << 7);
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(lsb);
}

void Instructions::Sla(uint8_t& op1) {
  bool msb = (op1 >> 7) & 0x1;
  op1 = (op1 << 1) & 0b11111110;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(msb);
}

void Instructions::Sra(uint8_t& op1) {
  uint8_t msb_u8 = op1 & 0b10000000;
  bool lsb = op1 & 0x1;
  op1 = (op1 >> 1) | msb_u8;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(lsb);
}

void Instructions::Swap(uint8_t& op1) {
  uint8_t lower_nibble = op1 & 0x0F;
  uint8_t higher_nibble = (op1 >> 4) & 0x0F;
  op1 = (lower_nibble << 4) | higher_nibble;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(false);
}

void Instructions::Srl(uint8_t& op1) {
  bool lsb = op1 & 0x1;
  op1 = (op1 >> 1) & 0b01111111;
  registers_.SetZeroFlag(op1, false);
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(false);
  registers_.SetCarryFlag(lsb);
}

void Instructions::Bit(int op1, uint8_t& op2) {
  registers_.SetZeroFlag((op2 >> op1) & 0x1, false);  // set if bit op1 of register op2 is 0
  registers_.SetSubtractionFlag(false);
  registers_.SetHalfCarryFlag(true);
}

void Instructions::Res(int op1, uint8_t& op2) {
  op2 &= ((1 << op1) ^ 0xFF);
}

void Instructions::Set(int op1, uint8_t& op2) {
  op2 |= (1 << op1);
}