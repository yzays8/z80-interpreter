# z80-interpreter

## About
This is a simple Z80 interpreter based on the Game Boy documents (easy to find and read). In other words, it is not an original Z80 interpreter and will have some bugs.
However, all of the [Blargg instruction tests](https://github.com/retrio/gb-test-roms/tree/master/cpu_instrs/individual) have passed, so the interpretation itself should be accurate.

Note that the STOP instruction is not implemented (it is not checked in the Blargg instruction tests). Also, the calculation of the number of clock cycles is likely to be incorrect.

## Usage
Prepare the file to be loaded in advance. By default, it is placed from address 0x0000 and 32KB Blargg test works in this way. Since memory banks are not supported at this time, large files can't be read. DO NOT use BIOS or game roms that you don't own, they are illegal.
```
make
make run FILE=<File>
```

## References
- https://izik1.github.io/gbops/index.html