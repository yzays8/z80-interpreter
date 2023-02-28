#include <cstring>
#include <fstream>
#include <iostream>

#include "mmu.hpp"

MMU::MMU() : memory_map_{0} {
}

void MMU::LoadFile(uint16_t base_addr, std::string path) {
  std::ifstream ifs(path, std::ios::binary | std::ios::in);
  if (!ifs.is_open()) {
    std::cerr << "Cannot open file stream" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  char data;
  for (int i = base_addr; ifs.get(data); ++i) {
    WriteByte(i, static_cast<uint8_t>(data));
  }
  std::cout << "Loaded File" << std::endl;
}

uint8_t& MMU::ReadByte(uint16_t addr) {
  return memory_map_[addr];
}

uint16_t MMU::ReadShort(uint16_t addr) const {
  // the value of higher address is higher bits
  return memory_map_[addr] | (memory_map_[addr + 1] << 8);
}

void MMU::WriteByte(uint16_t addr, uint8_t data) {
  memory_map_[addr] = data;
}

void MMU::WriteShort(uint16_t addr, uint16_t data) {
  memory_map_[addr] = data & 0xFF;
  memory_map_[addr + 1] = (data >> 8) & 0xFF;
}