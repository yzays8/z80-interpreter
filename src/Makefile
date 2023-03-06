CC = g++
TARGET = emu
OBJS = emu.o cpu.o mmu.o interrupt.o instructions.o timer.o registers.o

CXXFLAGS = -O3 -Wall

.PHONY: all
all: $(TARGET)

.PHONY: clean
clean:
	rm -rf *.o

.PHONY: run
run:
	./$(TARGET) ../$(FILE)

$(TARGET): $(OBJS) Makefile
	$(CC) $(OBJS) -o $@

%.o: %.cpp Makefile
	$(CC) $(CXXFLAGS) -c $<