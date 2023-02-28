#include <iostream>

#include "cpu.hpp"
#include "interrupt.hpp"
#include "instructions.hpp"
#include "registers.hpp"

CPU::CPU(Registers& registers, Instructions& instructions, MMU& mmu, Interrupt& interrupt)
    : registers_{registers}, instructions_{instructions}, mmu_{mmu}, interrupt_{interrupt},
      halt_{false} {
}

void CPU::TickCPU() {
  tcycles = 0;
  if (!halt_) {
    uint8_t opcode = mmu_.ReadByte(registers_.pc);
    ++registers_.pc;
    InterpretInstruction(opcode);
    tcycles += (tcycles_table_[opcode] + instructions_.GetBranchCycle());
    if (opcode == 0xCB) {
      tcycles += tcycles_table_cb_prefixed_[opcode];
    }
  } else {
    tcycles += 4;
  }

  int interrupt_type = interrupt_.CheckInterrupt();

  if (interrupt_type != NO_INTERRUPT) {
    // HALT waits for IF and IE to be non-zero and IME is not relevant
    halt_ = false;
  }

  if (interrupt_.GetIME()) {
    if (interrupt_type != NO_INTERRUPT) {
      interrupt_.SetIME(false);
      interrupt_.ResetIF(interrupt_type);
      registers_.sp -= 2;  // extend stack
      mmu_.WriteShort(registers_.sp, registers_.pc); // push pc to stack
      registers_.pc = kInterruptHandler[interrupt_type];
      tcycles += 20;
    }
  }
}

void CPU::DebugInstruction(const uint8_t opcode) {
  printf("PC: 0x%04X, Opcode: 0x%02X, AF: 0x%04X, BC: 0x%04X, DE: 0x%04X, HL: 0x%04X, SP: 0x%04X\n",
  registers_.pc-1, opcode, registers_.af.Get(), registers_.bc.Get(), registers_.de.Get(),
  registers_.hl.Get(), registers_.sp);
}

void CPU::InterpretInstruction(const uint8_t opcode) {
  // DebugInstruction(opcode);

  switch (opcode) {
    case 0x00:  // NOP
      break;
    case 0x01:  // LD BC, d16
      instructions_.Load(registers_.bc);
      break;
    case 0x02:  // LD (BC), A
      instructions_.Load(registers_.bc, registers_.a);
      break;
    case 0x03:  // INC BC
      instructions_.Inc(registers_.bc);
      break;
    case 0x04:  // INC B
      instructions_.Inc(registers_.b);
      break;
    case 0x05:  // DEC B
      instructions_.Dec(registers_.b);
      break;
    case 0x06:  // LD B, d8
      instructions_.Load(registers_.b);
      break;
    case 0x07:  // RLCA, rotate A left
      {
        bool msb = (registers_.a >> 7) & 0x1;
        registers_.a = (registers_.a << 1) | msb;
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        registers_.SetHalfCarryFlag(false);
        registers_.SetCarryFlag(msb);
      }
      break;
    case 0x08:  // LD (a16), SP
      mmu_.WriteShort(mmu_.ReadShort(registers_.pc), registers_.sp);
      registers_.pc += 2;
      break;
    case 0x09:  // ADD HL, BC
      instructions_.Add(registers_.hl, registers_.bc);
      break;
    case 0x0A:  // LD A, (BC)
      instructions_.Load(registers_.a, registers_.bc);
      break;
    case 0x0B:  // DEC BC
      instructions_.Dec(registers_.bc);
      break;
    case 0x0C:  // INC C
      instructions_.Inc(registers_.c);
      break;
    case 0x0D:  // DEC C
      instructions_.Dec(registers_.c);
      break;
    case 0x0E:  // LD C, d8
      instructions_.Load(registers_.c);
      break;
    case 0x0F:  // RRCA, rotate A right
      {
        bool lsb = registers_.a & 0x1;
        registers_.a = (registers_.a >> 1) | (lsb << 7);
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        registers_.SetHalfCarryFlag(false);
        registers_.SetCarryFlag(lsb);
      }
      break;
    case 0x10:  // STOP 0
      printf("STOP is not implemented: 0x%02X\n", opcode);
      std::exit(EXIT_FAILURE);
      break;
    case 0x11:  // LD DE, d16
      instructions_.Load(registers_.de);
      break;
    case 0x12:  // LD (DE), A
      instructions_.Load(registers_.de, registers_.a);
      break;
    case 0x13:  // INC DE
      instructions_.Inc(registers_.de);
      break;
    case 0x14:  // INC D
      instructions_.Inc(registers_.d);
      break;
    case 0x15:  // DEC D
      instructions_.Dec(registers_.d);
      break;
    case 0x16:  // LD D, d8
      instructions_.Load(registers_.d);
      break;
    case 0x17:  // RLA
      {
        bool msb = (registers_.a & 0x80) >> 7;
        registers_.a = (registers_.a << 1) | registers_.GetCarryFlag();
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        registers_.SetHalfCarryFlag(false);
        registers_.SetCarryFlag(msb);
      }
      break;
    case 0x18:  // JR r8
      instructions_.Jr(true);
      break;
    case 0x19:  // ADD HL, DE
      instructions_.Add(registers_.hl, registers_.de);
      break;
    case 0x1A:  // LD A, (DE)
      instructions_.Load(registers_.a, registers_.de);
      break;
    case 0x1B:  // DEC DE
      instructions_.Dec(registers_.de);
      break;
    case 0x1C:  // INC E
      instructions_.Inc(registers_.e);
      break;
    case 0x1D:  // DEC E
      instructions_.Dec(registers_.e);
      break;
    case 0x1E:  // LD E, d8
      instructions_.Load(registers_.e);
      break;
    case 0x1F:  // RRA
      {
        bool lsb = registers_.a & 0x1;
        registers_.a = (registers_.a >> 1) | (registers_.GetCarryFlag() << 7);
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        registers_.SetHalfCarryFlag(false);
        registers_.SetCarryFlag(lsb);
      }
      break;
    case 0x20:  // JR NZ, r8
      instructions_.Jr(!registers_.GetZeroFlag());
      break;
    case 0x21:  // LD HL, d16
      instructions_.Load(registers_.hl);
      break;
    case 0x22:  // LD (HL+), A
      instructions_.Load(registers_.hl, registers_.a);
      registers_.hl.Set(registers_.hl.Get() + 1);
      break;
    case 0x23:  // INC HL
      instructions_.Inc(registers_.hl);
      break;
    case 0x24:  // INC H
      instructions_.Inc(registers_.h);
      break;
    case 0x25:  // DEC H
      instructions_.Dec(registers_.h);
      break;
    case 0x26:  // LD H, d8
      instructions_.Load(registers_.h);
      break;
    case 0x27:  // DAA, convert the value of register A to the right BCD form based on the type of previous calculation
      if (!registers_.GetSubtractionFrag()) { //  if register A value is the result of addition
        if (registers_.GetCarryFlag() || (registers_.a > 0x99)) {
          registers_.a += 0x60;
          registers_.SetCarryFlag(true);
        }
        if (registers_.GetHalfCarryFlag() || ((registers_.a & 0xF) > 0x9)) {
          registers_.a += 0x6;
        }
      } else {  // if register A value is the result of subtraction
        if (registers_.GetCarryFlag()) {
          registers_.a -= 0x60;
        }
        if (registers_.GetHalfCarryFlag()) {
          registers_.a -= 0x6;
        }
      }
      registers_.SetZeroFlag(registers_.a, false);
      registers_.SetHalfCarryFlag(false);
      break;
    case 0x28:  // JR Z, r8
      instructions_.Jr(registers_.GetZeroFlag());
      break;
    case 0x29:  // ADD HL, HL
      instructions_.Add(registers_.hl, registers_.hl);
      break;
    case 0x2A:  // LD, A, (HL+)
      instructions_.Load(registers_.a, registers_.hl);
      registers_.hl.Set(registers_.hl.Get() + 1);
      break;
    case 0x2B:  // DEC HL
      instructions_.Dec(registers_.hl);
      break;
    case 0x2C:  // INC L
      instructions_.Inc(registers_.l);
      break;
    case 0x2D:  // DEC L
      instructions_.Dec(registers_.l);
      break;
    case 0x2E:  // LD L, d8
      instructions_.Load(registers_.l);
      break;
    case 0x2F:  // CPL, flip all bits
      registers_.a ^= 0xFF;
      registers_.SetSubtractionFlag(true);
      registers_.SetHalfCarryFlag(true);
      break;
    case 0x30:  // JR NC, r8
      instructions_.Jr(!registers_.GetCarryFlag());
      break;
    case 0x31:  // LD SP, d16
      registers_.sp = mmu_.ReadShort(registers_.pc);
      registers_.pc += 2;
      break;
    case 0x32:  // LD (HL-), A
      instructions_.Load(registers_.hl, registers_.a);
      registers_.hl.Set(registers_.hl.Get() - 1);
      break;
    case 0x33:  // INC SP
      ++registers_.sp;
      break;
    case 0x34:  // INC (HL)
      instructions_.Inc(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x35:  // DEC (HL)
      instructions_.Dec(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x36:  // LD (HL), d8
      mmu_.WriteByte(registers_.hl.Get(), mmu_.ReadByte(registers_.pc));
      ++registers_.pc;
      break;
    case 0x37:  // SCF
      registers_.SetSubtractionFlag(false);
      registers_.SetHalfCarryFlag(false);
      registers_.SetCarryFlag(true);
      break;
    case 0x38:  // JR C, r8
      instructions_.Jr(registers_.GetCarryFlag());
      break;
    case 0x39:  // ADD HL, SP
      {
        registers_.SetHalfCarryFlag(((registers_.hl.Get() & 0xFFF) + (registers_.sp & 0xFFF)) > 0xFFF);
        registers_.SetSubtractionFlag(false);
        uint32_t res = registers_.hl.Get() + registers_.sp;
        registers_.SetCarryFlag(res > 0xFFFF);
        registers_.hl.Set(static_cast<uint16_t>(res));
      }
      break;
    case 0x3A:  // LD A, (HL-)
      instructions_.Load(registers_.a, registers_.hl);
      registers_.hl.Set(registers_.hl.Get() - 1);
      break;
    case 0x3B:  // DEC SP
      --registers_.sp;
      break;
    case 0x3C:  // INC A
      instructions_.Inc(registers_.a);
      break;
    case 0x3D:  // DEC A
      instructions_.Dec(registers_.a);
      break;
    case 0x3E:  // LD A, d8
      instructions_.Load(registers_.a);
      break;
    case 0x3F:  // CCF
      registers_.SetSubtractionFlag(false);
      registers_.SetHalfCarryFlag(false);
      registers_.SetCarryFlag(!registers_.GetCarryFlag());
      break;
    case 0x40:  // LD B, B
      instructions_.Load(registers_.b, registers_.b);
      break;
    case 0x41:  // LD B, C
      instructions_.Load(registers_.b, registers_.c);
      break;
    case 0x42:  // LD B, D
      instructions_.Load(registers_.b, registers_.d);
      break;
    case 0x43:  // LD B, E
      instructions_.Load(registers_.b, registers_.e);
      break;
    case 0x44:  // LD B, H
      instructions_.Load(registers_.b, registers_.h);
      break;
    case 0x45:  // LD B, L
      instructions_.Load(registers_.b, registers_.l);
      break;
    case 0x46:  // LD B, (HL)
      instructions_.Load(registers_.b, registers_.hl);
      break;
    case 0x47:  // LD B, A
      instructions_.Load(registers_.b, registers_.a);
      break;
    case 0x48:  // LD C, B
      instructions_.Load(registers_.c, registers_.b);
      break;
    case 0x49:  // LD C, C
      instructions_.Load(registers_.c, registers_.c);
      break;
    case 0x4A:  // LD C, D
      instructions_.Load(registers_.c, registers_.d);
      break;
    case 0x4B:  // LD C, E
      instructions_.Load(registers_.c, registers_.e);
      break;
    case 0x4C:  // LD C, H
      instructions_.Load(registers_.c, registers_.h);
      break;
    case 0x4D:  // LD C, L
      instructions_.Load(registers_.c, registers_.l);
      break;
    case 0x4E:  // LD C, (HL)
      instructions_.Load(registers_.c, registers_.hl);
      break;
    case 0x4F:  // LD C, A
      instructions_.Load(registers_.c, registers_.a);
      break;
    case 0x50:  // LD D, B
      instructions_.Load(registers_.d, registers_.b);
      break;
    case 0x51:  // LD D, C
      instructions_.Load(registers_.d, registers_.c);
      break;
    case 0x52:  // LD D, D
      instructions_.Load(registers_.d, registers_.d);
      break;
    case 0x53:  // LD D, E
      instructions_.Load(registers_.d, registers_.e);
      break;
    case 0x54:  // LD D, H
      instructions_.Load(registers_.d, registers_.h);
      break;
    case 0x55:  // LD D, L
      instructions_.Load(registers_.d, registers_.l);
      break;
    case 0x56:  // LD D, (HL)
      instructions_.Load(registers_.d, registers_.hl);
      break;
    case 0x57:  // LD D, A
      instructions_.Load(registers_.d, registers_.a);
      break;
    case 0x58:  // LD E, B
      instructions_.Load(registers_.e, registers_.b);
      break;
    case 0x59:  // LD E, C
      instructions_.Load(registers_.e, registers_.c);
      break;
    case 0x5A:  // LD E, D
      instructions_.Load(registers_.e, registers_.d);
      break;
    case 0x5B:  // LD E, E
      instructions_.Load(registers_.e, registers_.e);
      break;
    case 0x5C:  // LD E, H
      instructions_.Load(registers_.e, registers_.h);
      break;
    case 0x5D:  // LD E, L
      instructions_.Load(registers_.e, registers_.l);
      break;
    case 0x5E:  // LD E, (HL)
      instructions_.Load(registers_.e, registers_.hl);
      break;
    case 0x5F:  // LD E, A
      instructions_.Load(registers_.e, registers_.a);
      break;
    case 0x60:  // LD H, B
      instructions_.Load(registers_.h, registers_.b);
      break;
    case 0x61:  // LD H, C
      instructions_.Load(registers_.h, registers_.c);
      break;
    case 0x62:  // LD H, D
      instructions_.Load(registers_.h, registers_.d);
      break;
    case 0x63:  // LD H, E
      instructions_.Load(registers_.h, registers_.e);
      break;
    case 0x64:  // LD H, H
      instructions_.Load(registers_.h, registers_.h);
      break;
    case 0x65:  // LD H, L
      instructions_.Load(registers_.h, registers_.l);
      break;
    case 0x66:  // LD H, (HL)
      instructions_.Load(registers_.h, registers_.hl);
      break;
    case 0x67:  // LD H, A
      instructions_.Load(registers_.h, registers_.a);
      break;
    case 0x68:  // LD L, B
      instructions_.Load(registers_.l, registers_.b);
      break;
    case 0x69:  // LD L, C
      instructions_.Load(registers_.l, registers_.c);
      break;
    case 0x6A:  // LD L, D
      instructions_.Load(registers_.l, registers_.d);
      break;
    case 0x6B:  // LD L, E
      instructions_.Load(registers_.l, registers_.e);
      break;
    case 0x6C:  // LD L, H
      instructions_.Load(registers_.l, registers_.h);
      break;
    case 0x6D:  // LD L, L
      instructions_.Load(registers_.l, registers_.l);
      break;
    case 0x6E:  // LD L, (HL)
      instructions_.Load(registers_.l, registers_.hl);
      break;
    case 0x6F:  // LD L, A
      instructions_.Load(registers_.l, registers_.a);
      break;
    case 0x70:  // LD (HL), B
      instructions_.Load(registers_.hl, registers_.b);
      break;
    case 0x71:  // LD (HL), C
      instructions_.Load(registers_.hl, registers_.c);
      break;
    case 0x72:  // LD (HL), D
      instructions_.Load(registers_.hl, registers_.d);
      break;
    case 0x73:  // LD (HL), E
      instructions_.Load(registers_.hl, registers_.e);
      break;
    case 0x74:  // LD (HL), H
      instructions_.Load(registers_.hl, registers_.h);
      break;
    case 0x75:  // LD (HL), L
      instructions_.Load(registers_.hl, registers_.l);
      break;
    case 0x76:  // HALT
      halt_= true;
      break;
    case 0x77:  // LD (HL), A
      instructions_.Load(registers_.hl, registers_.a);
      break;
    case 0x78:  // LD A, B
      instructions_.Load(registers_.a, registers_.b);
      break;
    case 0x79:  // LD A, C
      instructions_.Load(registers_.a, registers_.c);
      break;
    case 0x7A:  // LD A, D
      instructions_.Load(registers_.a, registers_.d);
      break;
    case 0x7B:  // LD A, E
      instructions_.Load(registers_.a, registers_.e);
      break;
    case 0x7C:  // LD A, H
      instructions_.Load(registers_.a, registers_.h);
      break;
    case 0x7D:  // LD A, L
      instructions_.Load(registers_.a, registers_.l);
      break;
    case 0x7E:  // LD A, (HL)
      instructions_.Load(registers_.a, registers_.hl);
      break;
    case 0x7F:  // LD A, A
      instructions_.Load(registers_.a, registers_.a);
      break;
    case 0x80:  // ADD A, B
      instructions_.Add(registers_.a, registers_.b);
      break;
    case 0x81:  // ADD A, C
      instructions_.Add(registers_.a, registers_.c);
      break;
    case 0x82:  // ADD A, D
      instructions_.Add(registers_.a, registers_.d);
      break;
    case 0x83:  // ADD A, E
      instructions_.Add(registers_.a, registers_.e);
      break;
    case 0x84:  // ADD A, H
      instructions_.Add(registers_.a, registers_.h);
      break;
    case 0x85:  // ADD A, L
      instructions_.Add(registers_.a, registers_.l);
      break;
    case 0x86:  // ADD A, (HL)
      instructions_.Add(registers_.a, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x87:  // ADD A, A
      instructions_.Add(registers_.a, registers_.a);
      break;
    case 0x88:  // ADC A, B
      instructions_.Adc(registers_.a, registers_.b);
      break;
    case 0x89:  // ADC A, C
      instructions_.Adc(registers_.a, registers_.c);
      break;
    case 0x8A:  // ADC A, D
      instructions_.Adc(registers_.a, registers_.d);
      break;
    case 0x8B:  // ADC A, E
      instructions_.Adc(registers_.a, registers_.e);
      break;
    case 0x8C:  // ADC A, H
      instructions_.Adc(registers_.a, registers_.h);
      break;
    case 0x8D:  // ADC A, L
      instructions_.Adc(registers_.a, registers_.l);
      break;
    case 0x8E:  // ADC A, (HL)
      instructions_.Adc(registers_.a, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x8F:  // ADC A, A
      instructions_.Adc(registers_.a, registers_.a);
      break;
    case 0x90:  // SUB A, B
      instructions_.Sub(registers_.b);
      break;
    case 0x91:  // SUB A, C
      instructions_.Sub(registers_.c);
      break;
    case 0x92:  // SUB A, D
      instructions_.Sub(registers_.d);
      break;
    case 0x93:  // SUB A, E
      instructions_.Sub(registers_.e);
      break;
    case 0x94:  // SUB A, H
      instructions_.Sub(registers_.h);
      break;
    case 0x95:  // SUB A, L
      instructions_.Sub(registers_.l);
      break;
    case 0x96:  // SUB A, (HL)
      instructions_.Sub(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x97:  // SUB A, A
      instructions_.Sub(registers_.a);
      break;
    case 0x98:  // SBC A, B
      instructions_.Sbc(registers_.a, registers_.b);
      break;
    case 0x99:  // SBC A, C
      instructions_.Sbc(registers_.a, registers_.c);
      break;
    case 0x9A:  // SBC A, D
      instructions_.Sbc(registers_.a, registers_.d);
      break;
    case 0x9B:  // SBC A, E
      instructions_.Sbc(registers_.a, registers_.e);
      break;
    case 0x9C:  // SBC A, H
      instructions_.Sbc(registers_.a, registers_.h);
      break;
    case 0x9D:  // SBC A, L
      instructions_.Sbc(registers_.a, registers_.l);
      break;
    case 0x9E:  // SBC A, (HL)
      instructions_.Sbc(registers_.a, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x9F:  // SBC A, A
      instructions_.Sbc(registers_.a, registers_.a);
      break;
    case 0xA0:  // AND B
      instructions_.And(registers_.b);
      break;
    case 0xA1:  // AND C
      instructions_.And(registers_.c);
      break;
    case 0xA2:  // AND D
      instructions_.And(registers_.d);
      break;
    case 0xA3:  // AND E
      instructions_.And(registers_.e);
      break;
    case 0xA4:  // AND H
      instructions_.And(registers_.h);
      break;
    case 0xA5:  // AND L
      instructions_.And(registers_.l);
      break;
    case 0xA6:  // AND (HL)
      instructions_.And(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xA7:  // AND A
      instructions_.And(registers_.a);
      break;
    case 0xA8:  // XOR B
      instructions_.Xor(registers_.b);
      break;
    case 0xA9:  // XOR C
      instructions_.Xor(registers_.c);
      break;
    case 0xAA:  // XOR D
      instructions_.Xor(registers_.d);
      break;
    case 0xAB:  // XOR E
      instructions_.Xor(registers_.e);
      break;
    case 0xAC:  // XOR H
      instructions_.Xor(registers_.h);
      break;
    case 0xAD:  // XOR L
      instructions_.Xor(registers_.l);
      break;
    case 0xAE:  // XOR (HL)
      instructions_.Xor(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xAF:  // XOR A
      instructions_.Xor(registers_.a);
      break;
    case 0xB0:  // OR B
      instructions_.Or(registers_.b);
      break;
    case 0xB1:  // OR C
      instructions_.Or(registers_.c);
      break;
    case 0xB2:  // OR D
      instructions_.Or(registers_.d);
      break;
    case 0xB3:  // OR E
      instructions_.Or(registers_.e);
      break;
    case 0xB4:  // OR H
      instructions_.Or(registers_.h);
      break;
    case 0xB5:  // OR L
      instructions_.Or(registers_.l);
      break;
    case 0xB6:  // OR (HL)
      instructions_.Or(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xB7:  // OR A
      instructions_.Or(registers_.a);
      break;
    case 0xB8:  // CP B
      instructions_.Cp(registers_.b);
      break;
    case 0xB9:  // CP C
      instructions_.Cp(registers_.c);
      break;
    case 0xBA:  // CP D
      instructions_.Cp(registers_.d);
      break;
    case 0xBB:  // CP E
      instructions_.Cp(registers_.e);
      break;
    case 0xBC:  // CP H
      instructions_.Cp(registers_.h);
      break;
    case 0xBD:  // CP L
      instructions_.Cp(registers_.l);
      break;
    case 0xBE:  // CP (HL)
      instructions_.Cp(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xBF:  // CP A
      instructions_.Cp(registers_.a);
      break;
    case 0xC0:  // RET NZ
      instructions_.Ret(!registers_.GetZeroFlag());
      break;
    case 0xC1:  // POP BC
      instructions_.Pop(registers_.bc);
      break;
    case 0xC2:  // JP NZ, a16
      instructions_.Jp(!registers_.GetZeroFlag());
      break;
    case 0xC3:  // JP a16
      instructions_.Jp(true);
      break;
    case 0xC4:  // CALL NZ, a16
      instructions_.Call(!registers_.GetZeroFlag());
      break;
    case 0xC5:  // PUSH BC
      instructions_.Push(registers_.bc);
      break;
    case 0xC6:  // ADD A, d8
      instructions_.Add(registers_.a, mmu_.ReadByte(registers_.pc++));
      break;
    case 0xC7:  // RST 00H
      instructions_.Rst(0x00);
      break;
    case 0xC8:  // RET Z
      instructions_.Ret(registers_.GetZeroFlag());
      break;
    case 0xC9:  // RET
      instructions_.Ret(true);
      break;
    case 0xCA:  // JP Z, a16
      instructions_.Jp(registers_.GetZeroFlag());
      break;
    case 0xCB:  // PREFIX CB
      InterpretInstructionEx(mmu_.ReadByte(registers_.pc++));
      break;
    case 0xCC:  // CALL Z, a16
      instructions_.Call(registers_.GetZeroFlag());
      break;
    case 0xCD:  // CALL a16
      instructions_.Call(true);
      break;
    case 0xCE:  // ADC A, d8
      instructions_.Adc(registers_.a, mmu_.ReadByte(registers_.pc++));
      break;
    case 0xCF:  // RST 08H
      instructions_.Rst(0x08);
      break;
    case 0xD0:  // RET NC
      instructions_.Ret(!registers_.GetCarryFlag());
      break;
    case 0xD1:  // POP DE
      instructions_.Pop(registers_.de);
      break;
    case 0xD2:  // JP NC, a16
      instructions_.Jp(!registers_.GetCarryFlag());
      break;
    case 0xD4:  // CALL NC, a16
      instructions_.Call(!registers_.GetCarryFlag());
      break;
    case 0xD5:  // PUSH DE
      instructions_.Push(registers_.de);
      break;
    case 0xD6:  // SUB A, d8
      instructions_.Sub(mmu_.ReadByte(registers_.pc++));
      break;
    case 0xD7:  // RST 10H
      instructions_.Rst(0x10);
      break;
    case 0xD8:  // RET C
      instructions_.Ret(registers_.GetCarryFlag());
      break;
    case 0xD9:  // RETI
      instructions_.Ret(true);
      interrupt_.SetIME(true);
      break;
    case 0xDA:  // JP C, a16
      instructions_.Jp(registers_.GetCarryFlag());
      break;
    case 0xDC:  // CALL C, a16
      instructions_.Call(registers_.GetCarryFlag());
      break;
    case 0xDE:  // SBC A, d8
      instructions_.Sbc(registers_.a, mmu_.ReadByte(registers_.pc));
      ++registers_.pc;
      break;
    case 0xDF:  // RST 18H
      instructions_.Rst(0x18);
      break;
    case 0xE0:  // LDH (a8), A
      mmu_.WriteByte(0xFF00 + mmu_.ReadByte(registers_.pc), registers_.a);
      ++registers_.pc;
      break;
    case 0xE1:  // POP HL
      instructions_.Pop(registers_.hl);
      break;
    case 0xE2:  // LD (C), A
      mmu_.WriteByte(0xFF00 + registers_.c, registers_.a);
      break;
    case 0xE5:  // PUSH HL
      instructions_.Push(registers_.hl);
      break;
    case 0xE6:  // AND d8
      instructions_.And(mmu_.ReadByte(registers_.pc++));
      break;
    case 0xE7:  // RST 20H
      instructions_.Rst(0x20);
      break;
    case 0xE8:  // ADD SP, r8
      {
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        int8_t r8 = static_cast<int8_t>(mmu_.ReadByte(registers_.pc));
        ++registers_.pc;
        registers_.SetHalfCarryFlag((registers_.sp & 0xF) + (r8 & 0xF) > 0xF);
        registers_.SetCarryFlag((registers_.sp & 0xFF) + (r8 & 0xFF) > 0xFF); // (r8 & 0xFF) is not r8
        registers_.sp += r8;
      }
      break;
    case 0xE9:  // JP HL, some document say JP (HL) but it is a mistake
      registers_.pc = registers_.hl.Get();
      break;
    case 0xEA:  // LD (a16), A
      mmu_.WriteByte(mmu_.ReadShort(registers_.pc), registers_.a);
      registers_.pc += 2;
      break;
    case 0xEE:  // XOR d8
      registers_.a ^= mmu_.ReadByte(registers_.pc);
      ++registers_.pc;
      registers_.SetZeroFlag(registers_.a, false);
      registers_.SetSubtractionFlag(false);
      registers_.SetHalfCarryFlag(false);
      registers_.SetCarryFlag(false);
      break;
    case 0xEF:  // RST 28H
      instructions_.Rst(0x28);
      break;
    case 0xF0:  // LDH A, (a8)
      registers_.a = mmu_.ReadByte(0xFF00 + mmu_.ReadByte(registers_.pc));  // check
      ++registers_.pc;
      break;
    case 0xF1:  // POP AF
      registers_.af.Set(mmu_.ReadShort(registers_.sp) & 0xFFF0);
      registers_.sp += 2;
      break;
    case 0xF2:  // LD A, (C)
      registers_.a = mmu_.ReadByte(0xFF00 + registers_.c);
      break;
    case 0xF3:  // DI
      interrupt_.SetIME(false);
      break;
    case 0xF5:  // PUSH AF
      instructions_.Push(registers_.af);
      break;
    case 0xF6:  // OR d8
      instructions_.Or(mmu_.ReadByte(registers_.pc++));
      break;
    case 0xF7:  // RST 30H
      instructions_.Rst(0x30);
      break;
    case 0xF8:  // LD HL, SP+r8
      {
        registers_.SetZeroFlag(0, true);
        registers_.SetSubtractionFlag(false);
        int8_t r8 = static_cast<int8_t>(mmu_.ReadByte(registers_.pc));
        ++registers_.pc;
        registers_.SetHalfCarryFlag((registers_.sp & 0xF) + (r8 & 0xF) > 0xF);
        registers_.SetCarryFlag((registers_.sp & 0xFF) + (r8 & 0xFF) > 0xFF);
        registers_.hl.Set(registers_.sp + r8);
      }
      break;
    case 0xF9:  // LD SP, HL
      registers_.sp = registers_.hl.Get();
      break;
    case 0xFA:  // LD A, (a16)
      registers_.a = mmu_.ReadByte(mmu_.ReadShort(registers_.pc));
      registers_.pc += 2;
      break;
    case 0xFB:  // EI
      interrupt_.SetIME(true);
      break;
    case 0xFE:  // CP d8
      instructions_.Cp(mmu_.ReadByte(registers_.pc++));
      break;
    case 0xFF:  // RST 38H
      instructions_.Rst(0x38);
      break;
    default:
      printf("not implement: 0x%02X\n", opcode);
      std::exit(EXIT_FAILURE);
      break;

  }
}

void CPU::InterpretInstructionEx(const uint8_t opcode) {
  // DebugInstruction(opcode);

  switch (opcode) {
    case 0x00:  // RLC B
      instructions_.Rlc(registers_.b);
      break;
    case 0x01:  // RLC C
      instructions_.Rlc(registers_.c);
      break;
    case 0x02:  // RLC D
      instructions_.Rlc(registers_.d);
      break;
    case 0x03:  // RLC E
      instructions_.Rlc(registers_.e);
      break;
    case 0x04:  // RLC H
      instructions_.Rlc(registers_.h);
      break;
    case 0x05:  // RLC L
      instructions_.Rlc(registers_.l);
      break;
    case 0x06:  // RLC (HL)
      instructions_.Rlc(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x07:  // RLC A
      instructions_.Rlc(registers_.a);
      break;
    case 0x08:  // RRC B
      instructions_.Rrc(registers_.b);
      break;
    case 0x09:  // RRC C
      instructions_.Rrc(registers_.c);
      break;
    case 0x0A:  // RRC D
      instructions_.Rrc(registers_.d);
      break;
    case 0x0B:  // RRC E
      instructions_.Rrc(registers_.e);
      break;
    case 0x0C:  // RRC H
      instructions_.Rrc(registers_.h);
      break;
    case 0x0D:  // RRC L
      instructions_.Rrc(registers_.l);
      break;
    case 0x0E:  // RRC (HL)
      instructions_.Rrc(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x0F:  // RRC A
      instructions_.Rrc(registers_.a);
      break;
    case 0x10:  // RL B
      instructions_.Rl(registers_.b);
      break;
    case 0x11:  // RL C
      instructions_.Rl(registers_.c);
      break;
    case 0x12:  // RL D
      instructions_.Rl(registers_.d);
      break;
    case 0x13:  // RL E
      instructions_.Rl(registers_.e);
      break;
    case 0x14:  // RL H
      instructions_.Rl(registers_.h);
      break;
    case 0x15:  // RL L
      instructions_.Rl(registers_.l);
      break;
    case 0x16:  // RL (HL)
      instructions_.Rl(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x17:  // RL A
      instructions_.Rl(registers_.a);
      break;
    case 0x18:  // RR B
      instructions_.Rr(registers_.b);
      break;
    case 0x19:  // RR C
      instructions_.Rr(registers_.c);
      break;
    case 0x1A:  // RR D
      instructions_.Rr(registers_.d);
      break;
    case 0x1B:  // RR E
      instructions_.Rr(registers_.e);
      break;
    case 0x1C:  // RR H
      instructions_.Rr(registers_.h);
      break;
    case 0x1D:  // RR L
      instructions_.Rr(registers_.l);
      break;
    case 0x1E:  // RR (HL)
      instructions_.Rr(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x1F:  // RR A
      instructions_.Rr(registers_.a);
      break;
    case 0x20:  // SLA B
      instructions_.Sla(registers_.b);
      break;
    case 0x21:  // SLA C
      instructions_.Sla(registers_.c);
      break;
    case 0x22:  // SLA D
      instructions_.Sla(registers_.d);
      break;
    case 0x23:  // SLA E
      instructions_.Sla(registers_.e);
      break;
    case 0x24:  // SLA H
      instructions_.Sla(registers_.h);
      break;
    case 0x25:  // SLA L
      instructions_.Sla(registers_.l);
      break;
    case 0x26:  // SLA (HL)
      instructions_.Sla(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x27:  // SLA A
      instructions_.Sla(registers_.a);
      break;
    case 0x28:  // SRA B
      instructions_.Sra(registers_.b);
      break;
    case 0x29:  // SRA C
      instructions_.Sra(registers_.c);
      break;
    case 0x2A:  // SRA D
      instructions_.Sra(registers_.d);
      break;
    case 0x2B:  // SRA E
      instructions_.Sra(registers_.e);
      break;
    case 0x2C:  // SRA H
      instructions_.Sra(registers_.h);
      break;
    case 0x2D:  // SRA L
      instructions_.Sra(registers_.l);
      break;
    case 0x2E:  // SRA (HL)
      instructions_.Sra(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x2F:  // SRA A
      instructions_.Sra(registers_.a);
      break;
    case 0x30:  // SWAP B
      instructions_.Swap(registers_.b);
      break;
    case 0x31:  // SWAP C
      instructions_.Swap(registers_.c);
      break;
    case 0x32:  // SWAP D
      instructions_.Swap(registers_.d);
      break;
    case 0x33:  // SWAP E
      instructions_.Swap(registers_.e);
      break;
    case 0x34:  // SWAP H
      instructions_.Swap(registers_.h);
      break;
    case 0x35:  // SWAP L
      instructions_.Swap(registers_.l);
      break;
    case 0x36:  // SWAP (HL)
      instructions_.Swap(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x37:  // SWAP A
      instructions_.Swap(registers_.a);
      break;
    case 0x38:  // SRL B
      instructions_.Srl(registers_.b);
      break;
    case 0x39:  // SRL C
      instructions_.Srl(registers_.c);
      break;
    case 0x3A:  // SRL D
      instructions_.Srl(registers_.d);
      break;
    case 0x3B:  // SRL E
      instructions_.Srl(registers_.e);
      break;
    case 0x3C:  // SRL H
      instructions_.Srl(registers_.h);
      break;
    case 0x3D:  // SRL L
      instructions_.Srl(registers_.l);
      break;
    case 0x3E:  // SRL (HL)
      instructions_.Srl(mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x3F:  // SRL A
      instructions_.Srl(registers_.a);
      break;
    case 0x40:  // BIT 0, B
      instructions_.Bit(0, registers_.b);
      break;
    case 0x41:  // BIT 0, C
      instructions_.Bit(0, registers_.c);
      break;
    case 0x42:  // BIT 0, D
      instructions_.Bit(0, registers_.d);
      break;
    case 0x43:  // BIT 0, E
      instructions_.Bit(0, registers_.e);
      break;
    case 0x44:  // BIT 0, H
      instructions_.Bit(0, registers_.h);
      break;
    case 0x45:  // BIT 0, L
      instructions_.Bit(0, registers_.l);
      break;
    case 0x46:  // BIT 0, (HL)
      instructions_.Bit(0, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x47:  // BIT 0, A
      instructions_.Bit(0, registers_.a);
      break;
    case 0x48:  // BIT 1, B
      instructions_.Bit(1, registers_.b);
      break;
    case 0x49:  // BIT 1, C
      instructions_.Bit(1, registers_.c);
      break;
    case 0x4A:  // BIT 1, D
      instructions_.Bit(1, registers_.d);
      break;
    case 0x4B:  // BIT 1, E
      instructions_.Bit(1, registers_.e);
      break;
    case 0x4C:  // BIT 1, H
      instructions_.Bit(1, registers_.h);
      break;
    case 0x4D:  // BIT 1, L
      instructions_.Bit(1, registers_.l);
      break;
    case 0x4E:  // BIT 1, (HL)
      instructions_.Bit(1, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x4F:  // BIT 1, A
      instructions_.Bit(1, registers_.a);
      break;
    case 0x50:  // BIT 2, B
      instructions_.Bit(2, registers_.b);
      break;
    case 0x51:  // BIT 2, C
      instructions_.Bit(2, registers_.c);
      break;
    case 0x52:  // BIT 2, D
      instructions_.Bit(2, registers_.d);
      break;
    case 0x53:  // BIT 2, E
      instructions_.Bit(2, registers_.e);
      break;
    case 0x54:  // BIT 2, H
      instructions_.Bit(2, registers_.h);
      break;
    case 0x55:  // BIT 2, L
      instructions_.Bit(2, registers_.l);
      break;
    case 0x56:  // BIT 2, (HL)
      instructions_.Bit(2, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x57:  // BIT 2, A
      instructions_.Bit(2, registers_.a);
      break;
    case 0x58:  // BIT 3, B
      instructions_.Bit(3, registers_.b);
      break;
    case 0x59:  // BIT 3, C
      instructions_.Bit(3, registers_.c);
      break;
    case 0x5A:  // BIT 3, D
      instructions_.Bit(3, registers_.d);
      break;
    case 0x5B:  // BIT 3, E
      instructions_.Bit(3, registers_.e);
      break;
    case 0x5C:  // BIT 3, H
      instructions_.Bit(3, registers_.h);
      break;
    case 0x5D:  // BIT 3, L
      instructions_.Bit(3, registers_.l);
      break;
    case 0x5E:  // BIT 3, (HL)
      instructions_.Bit(3, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x5F:  // BIT 3, A
      instructions_.Bit(3, registers_.a);
      break;
    case 0x60:  // BIT 4, B
      instructions_.Bit(4, registers_.b);
      break;
    case 0x61:  // BIT 4, C
      instructions_.Bit(4, registers_.c);
      break;
    case 0x62:  // BIT 4, D
      instructions_.Bit(4, registers_.d);
      break;
    case 0x63:  // BIT 4, E
      instructions_.Bit(4, registers_.e);
      break;
    case 0x64:  // BIT 4, H
      instructions_.Bit(4, registers_.h);
      break;
    case 0x65:  // BIT 4, L
      instructions_.Bit(4, registers_.l);
      break;
    case 0x66:  // BIT 4, (HL)
      instructions_.Bit(4, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x67:  // BIT 4, A
      instructions_.Bit(4, registers_.a);
      break;
    case 0x68:  // BIT 5, B
      instructions_.Bit(5, registers_.b);
      break;
    case 0x69:  // BIT 5, C
      instructions_.Bit(5, registers_.c);
      break;
    case 0x6A:  // BIT 5, D
      instructions_.Bit(5, registers_.d);
      break;
    case 0x6B:  // BIT 5, E
      instructions_.Bit(5, registers_.e);
      break;
    case 0x6C:  // BIT 5, H
      instructions_.Bit(5, registers_.h);
      break;
    case 0x6D:  // BIT 5, L
      instructions_.Bit(5, registers_.l);
      break;
    case 0x6E:  // BIT 5, (HL)
      instructions_.Bit(5, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x6F:  // BIT 5, A
      instructions_.Bit(5, registers_.a);
      break;
    case 0x70:  // BIT 6, B
      instructions_.Bit(6, registers_.b);
      break;
    case 0x71:  // BIT 6, C
      instructions_.Bit(6, registers_.c);
      break;
    case 0x72:  // BIT 6, D
      instructions_.Bit(6, registers_.d);
      break;
    case 0x73:  // BIT 6, E
      instructions_.Bit(6, registers_.e);
      break;
    case 0x74:  // BIT 6, H
      instructions_.Bit(6, registers_.h);
      break;
    case 0x75:  // BIT 6, L
      instructions_.Bit(6, registers_.l);
      break;
    case 0x76:  // BIT 6, (HL)
      instructions_.Bit(6, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x77:  // BIT 6, A
      instructions_.Bit(6, registers_.a);
      break;
    case 0x78:  // BIT 7, B
      instructions_.Bit(7, registers_.b);
      break;
    case 0x79:  // BIT 7, C
      instructions_.Bit(7, registers_.c);
      break;
    case 0x7A:  // BIT 7, D
      instructions_.Bit(7, registers_.d);
      break;
    case 0x7B:  // BIT 7, E
      instructions_.Bit(7, registers_.e);
      break;
    case 0x7C:  // BIT 7, H
      instructions_.Bit(7, registers_.h);
      break;
    case 0x7D:  // BIT 7, L
      instructions_.Bit(7, registers_.l);
      break;
    case 0x7E:  // BIT 7, (HL)
      instructions_.Bit(7, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x7F:  // BIT 7, A
      instructions_.Bit(7, registers_.a);
      break;
    case 0x80:  // RES 0, B
      instructions_.Res(0, registers_.b);
      break;
    case 0x81:  // RES 0, C
      instructions_.Res(0, registers_.c);
      break;
    case 0x82:  // RES 0, D
      instructions_.Res(0, registers_.d);
      break;
    case 0x83:  // RES 0, E
      instructions_.Res(0, registers_.e);
      break;
    case 0x84:  // RES 0, H
      instructions_.Res(0, registers_.h);
      break;
    case 0x85:  // RES 0, L
      instructions_.Res(0, registers_.l);
      break;
    case 0x86:  // RES 0, (HL)
      instructions_.Res(0, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x87:  // RES 0, A
      instructions_.Res(0, registers_.a);
      break;
    case 0x88:  // RES 1, B
      instructions_.Res(1, registers_.b);
      break;
    case 0x89:  // RES 1, C
      instructions_.Res(1, registers_.c);
      break;
    case 0x8A:  // RES 1, D
      instructions_.Res(1, registers_.d);
      break;
    case 0x8B:  // RES 1, E
      instructions_.Res(1, registers_.e);
      break;
    case 0x8C:  // RES 1, H
      instructions_.Res(1, registers_.h);
      break;
    case 0x8D:  // RES 1, L
      instructions_.Res(1, registers_.l);
      break;
    case 0x8E:  // RES 1, (HL)
      instructions_.Res(1, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x8F:  // RES 1, A
      instructions_.Res(1, registers_.a);
      break;
    case 0x90:  // RES 2, B
      instructions_.Res(2, registers_.b);
      break;
    case 0x91:  // RES 2, C
      instructions_.Res(2, registers_.c);
      break;
    case 0x92:  // RES 2, D
      instructions_.Res(2, registers_.d);
      break;
    case 0x93:  // RES 2, E
      instructions_.Res(2, registers_.e);
      break;
    case 0x94:  // RES 2, H
      instructions_.Res(2, registers_.h);
      break;
    case 0x95:  // RES 2, L
      instructions_.Res(2, registers_.l);
      break;
    case 0x96:  // RES 2, (HL)
      instructions_.Res(2, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x97:  // RES 2, A
      instructions_.Res(2, registers_.a);
      break;
    case 0x98:  // RES 3, B
      instructions_.Res(3, registers_.b);
      break;
    case 0x99:  // RES 3, C
      instructions_.Res(3, registers_.c);
      break;
    case 0x9A:  // RES 3, D
      instructions_.Res(3, registers_.d);
      break;
    case 0x9B:  // RES 3, E
      instructions_.Res(3, registers_.e);
      break;
    case 0x9C:  // RES 3, H
      instructions_.Res(3, registers_.h);
      break;
    case 0x9D:  // RES 3, L
      instructions_.Res(3, registers_.l);
      break;
    case 0x9E:  // RES 3, (HL)
      instructions_.Res(3, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0x9F:  // RES 3, A
      instructions_.Res(3, registers_.a);
      break;
    case 0xA0:  // RES 4, B
      instructions_.Res(4, registers_.b);
      break;
    case 0xA1:  // RES 4, C
      instructions_.Res(4, registers_.c);
      break;
    case 0xA2:  // RES 4, D
      instructions_.Res(4, registers_.d);
      break;
    case 0xA3:  // RES 4, E
      instructions_.Res(4, registers_.e);
      break;
    case 0xA4:  // RES 4, H
      instructions_.Res(4, registers_.h);
      break;
    case 0xA5:  // RES 4, L
      instructions_.Res(4, registers_.l);
      break;
    case 0xA6:  // RES 4, (HL)
      instructions_.Res(4, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xA7:  // RES 4, A
      instructions_.Res(4, registers_.a);
      break;
    case 0xA8:  // RES 5, B
      instructions_.Res(5, registers_.b);
      break;
    case 0xA9:  // RES 5, C
      instructions_.Res(5, registers_.c);
      break;
    case 0xAA:  // RES 5, D
      instructions_.Res(5, registers_.d);
      break;
    case 0xAB:  // RES 5, E
      instructions_.Res(5, registers_.e);
      break;
    case 0xAC:  // RES 5, H
      instructions_.Res(5, registers_.h);
      break;
    case 0xAD:  // RES 5, L
      instructions_.Res(5, registers_.l);
      break;
    case 0xAE:  // RES 5, (HL)
      instructions_.Res(5, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xAF:  // RES 5, A
      instructions_.Res(5, registers_.a);
      break;
    case 0xB0:  // RES 6, B
      instructions_.Res(6, registers_.b);
      break;
    case 0xB1:  // RES 6, C
      instructions_.Res(6, registers_.c);
      break;
    case 0xB2:  // RES 6, D
      instructions_.Res(6, registers_.d);
      break;
    case 0xB3:  // RES 6, E
      instructions_.Res(6, registers_.e);
      break;
    case 0xB4:  // RES 6, H
      instructions_.Res(6, registers_.h);
      break;
    case 0xB5:  // RES 6, L
      instructions_.Res(6, registers_.l);
      break;
    case 0xB6:  // RES 6, (HL)
      instructions_.Res(6, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xB7:  // RES 6, A
      instructions_.Res(6, registers_.a);
      break;
    case 0xB8:  // RES 7, B
      instructions_.Res(7, registers_.b);
      break;
    case 0xB9:  // RES 7, C
      instructions_.Res(7, registers_.c);
      break;
    case 0xBA:  // RES 7, D
      instructions_.Res(7, registers_.d);
      break;
    case 0xBB:  // RES 7, E
      instructions_.Res(7, registers_.e);
      break;
    case 0xBC:  // RES 7, H
      instructions_.Res(7, registers_.h);
      break;
    case 0xBD:  // RES 7, L
      instructions_.Res(7, registers_.l);
      break;
    case 0xBE:  // RES 7, (HL)
      instructions_.Res(7, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xBF:  // RES 7, A
      instructions_.Res(7, registers_.a);
      break;
    case 0xC0:  // SET 0, B
      instructions_.Set(0, registers_.b);
      break;
    case 0xC1:  // SET 0, C
      instructions_.Set(0, registers_.c);
      break;
    case 0xC2:  // SET 0, D
      instructions_.Set(0, registers_.d);
      break;
    case 0xC3:  // SET 0, E
      instructions_.Set(0, registers_.e);
      break;
    case 0xC4:  // SET 0, H
      instructions_.Set(0, registers_.h);
      break;
    case 0xC5:  // SET 0, L
      instructions_.Set(0, registers_.l);
      break;
    case 0xC6:  // SET 0, (HL)
      instructions_.Set(0, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xC7:  // SET 0, A
      instructions_.Set(0, registers_.a);
      break;
    case 0xC8:  // SET 1, B
      instructions_.Set(1, registers_.b);
      break;
    case 0xC9:  // SET 1, C
      instructions_.Set(1, registers_.c);
      break;
    case 0xCA:  // SET 1, D
      instructions_.Set(1, registers_.d);
      break;
    case 0xCB:  // SET 1, E
      instructions_.Set(1, registers_.e);
      break;
    case 0xCC:  // SET 1, H
      instructions_.Set(1, registers_.h);
      break;
    case 0xCD:  // SET 1, L
      instructions_.Set(1, registers_.l);
      break;
    case 0xCE:  // SET 1, (HL)
      instructions_.Set(1, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xCF:  // SET 1, A
      instructions_.Set(1, registers_.a);
      break;
    case 0xD0:  // SET 2, B
      instructions_.Set(2, registers_.b);
      break;
    case 0xD1:  // SET 2, C
      instructions_.Set(2, registers_.c);
      break;
    case 0xD2:  // SET 2, D
      instructions_.Set(2, registers_.d);
      break;
    case 0xD3:  // SET 2, E
      instructions_.Set(2, registers_.e);
      break;
    case 0xD4:  // SET 2, H
      instructions_.Set(2, registers_.h);
      break;
    case 0xD5:  // SET 2, L
      instructions_.Set(2, registers_.l);
      break;
    case 0xD6:  // SET 2, (HL)
      instructions_.Set(2, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xD7:  // SET 2, A
      instructions_.Set(2, registers_.a);
      break;
    case 0xD8:  // SET 3, B
      instructions_.Set(3, registers_.b);
      break;
    case 0xD9:  // SET 3, C
      instructions_.Set(3, registers_.c);
      break;
    case 0xDA:  // SET 3, D
      instructions_.Set(3, registers_.d);
      break;
    case 0xDB:  // SET 3, E
      instructions_.Set(3, registers_.e);
      break;
    case 0xDC:  // SET 3, H
      instructions_.Set(3, registers_.h);
      break;
    case 0xDD:  // SET 3, L
      instructions_.Set(3, registers_.l);
      break;
    case 0xDE:  // SET 3, (HL)
      instructions_.Set(3, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xDF:  // SET 3, A
      instructions_.Set(3, registers_.a);
      break;
    case 0xE0:  // SET 4, B
      instructions_.Set(4, registers_.b);
      break;
    case 0xE1:  // SET 4, C
      instructions_.Set(4, registers_.c);
      break;
    case 0xE2:  // SET 4, D
      instructions_.Set(4, registers_.d);
      break;
    case 0xE3:  // SET 4, E
      instructions_.Set(4, registers_.e);
      break;
    case 0xE4:  // SET 4, H
      instructions_.Set(4, registers_.h);
      break;
    case 0xE5:  // SET 4, L
      instructions_.Set(4, registers_.l);
      break;
    case 0xE6:  // SET 4, (HL)
      instructions_.Set(4, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xE7:  // SET 4, A
      instructions_.Set(4, registers_.a);
      break;
    case 0xE8:  // SET 5, B
      instructions_.Set(5, registers_.b);
      break;
    case 0xE9:  // SET 5, C
      instructions_.Set(5, registers_.c);
      break;
    case 0xEA:  // SET 5, D
      instructions_.Set(5, registers_.d);
      break;
    case 0xEB:  // SET 5, E
      instructions_.Set(5, registers_.e);
      break;
    case 0xEC:  // SET 5, H
      instructions_.Set(5, registers_.h);
      break;
    case 0xED:  // SET 5, L
      instructions_.Set(5, registers_.l);
      break;
    case 0xEE:  // SET 5, (HL)
      instructions_.Set(5, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xEF:  // SET 5, A
      instructions_.Set(5, registers_.a);
      break;
    case 0xF0:  // SET 6, B
      instructions_.Set(6, registers_.b);
      break;
    case 0xF1:  // SET 6, C
      instructions_.Set(6, registers_.c);
      break;
    case 0xF2:  // SET 6, D
      instructions_.Set(6, registers_.d);
      break;
    case 0xF3:  // SET 6, E
      instructions_.Set(6, registers_.e);
      break;
    case 0xF4:  // SET 6, H
      instructions_.Set(6, registers_.h);
      break;
    case 0xF5:  // SET 6, L
      instructions_.Set(6, registers_.l);
      break;
    case 0xF6:  // SET 6, (HL)
      instructions_.Set(6, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xF7:  // SET 6, A
      instructions_.Set(6, registers_.a);
      break;
    case 0xF8:  // SET 7, B
      instructions_.Set(7, registers_.b);
      break;
    case 0xF9:  // SET 7, C
      instructions_.Set(7, registers_.c);
      break;
    case 0xFA:  // SET 7, D
      instructions_.Set(7, registers_.d);
      break;
    case 0xFB:  // SET 7, E
      instructions_.Set(7, registers_.e);
      break;
    case 0xFC:  // SET 7, H
      instructions_.Set(7, registers_.h);
      break;
    case 0xFD:  // SET 7, L
      instructions_.Set(7, registers_.l);
      break;
    case 0xFE:  // SET 7, (HL)
      instructions_.Set(7, mmu_.ReadByte(registers_.hl.Get()));
      break;
    case 0xFF:  // SET 7, A
      instructions_.Set(7, registers_.a);
      break;
    default:
      printf("Not implemented CB-Prefixed instruction: %02X\n", opcode);
      std::exit(EXIT_FAILURE);
      break;
  }
}

