#pragma once

#include <array>
#include <string>

class MMU {
 public:
  MMU();
  void LoadFile(uint16_t base_addr, std::string path);
	uint8_t& ReadByte(uint16_t addr);
	uint16_t ReadShort(uint16_t addr) const;	// read 16bits
	void WriteByte(uint16_t addr, uint8_t data);
	void WriteShort(uint16_t addr, uint16_t data);	// write 16bits

 private:
  uint8_t memory_map_[0x10000];
};