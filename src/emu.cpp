#include <iostream>

#include "mmu.hpp"
#include "cpu.hpp"
#include "registers.hpp"
#include "instructions.hpp"
#include "interrupt.hpp"
#include "timer.hpp"

void CheckSerialPort(MMU& mmu) {
  if (mmu.ReadByte(0xFF02) == 0x81) {
    char c = mmu.ReadByte(0xFF01);
    printf("%c", c);
    mmu.WriteByte(0xFF02, 0x0);
  }
}

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Arguments error" << std::endl;
    return EXIT_FAILURE;
  }

  Registers registers{};
  MMU mmu{};
  Interrupt interrupt{mmu};
  Instructions instructions{registers, mmu};
  CPU cpu{registers, instructions, mmu, interrupt};
  Timer timer{cpu, mmu, interrupt};

  mmu.LoadFile(0x0000, argv[1]);

  for (;;) {
    cpu.TickCPU();
    timer.TickTimer();
    CheckSerialPort(mmu);
  }

  return 0;
}