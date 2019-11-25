#include "port_expander_brl4.h"

#define SET_CS    {mPORTBSetBits(BIT_0);}
#define CLEAR_CS  {mPORTBClearBits(BIT_0);}

void initPE() {
  writePE(IOCON, ( CLEAR_BANK   | CLEAR_MIRROR | SET_SEQOP |
                   CLEAR_DISSLW | CLEAR_HAEN   | CLEAR_ODR |
                   CLEAR_INTPOL ));
}

void clearBits(unsigned char addr, unsigned char bitmask){
  if (addr <= 0x15){
    unsigned char cur_val = readPE(addr);
    writePE(addr, cur_val & (~bitmask));
  }
}

void setBits(unsigned char addr, unsigned char bitmask){
  if (addr <= 0x15){
    unsigned char cur_val = readPE(addr);
    writePE(addr, cur_val | (bitmask));
  }
}

void toggleBits(unsigned char addr, unsigned char bitmask){
  if (addr <= 0x15){
    unsigned char cur_val = readPE(addr);
    writePE(addr, cur_val ^ (bitmask));
  }
}

unsigned char readBits(unsigned char addr, unsigned char bitmask){
  if (addr <= 0x15){
    unsigned char cur_val = readPE(addr) & bitmask ;
    return cur_val ;
  }
}

void mPortYSetPinsOut(unsigned char bitmask){
  clearBits(IODIRY, bitmask);
}

void mPortZSetPinsOut(unsigned char bitmask){
  clearBits(IODIRZ, bitmask);
}

void mPortYSetPinsIn(unsigned char bitmask){
  setBits(IODIRY, bitmask);
}

void mPortZSetPinsIn(unsigned char bitmask){
  setBits(IODIRZ, bitmask);
}

void mPortYIntEnable(unsigned char bitmask){
  setBits(GPINTENY, bitmask);
}

void mPortZIntEnable(unsigned char bitmask){
  setBits(GPINTENZ, bitmask);
}

void mPortYIntDisable(unsigned char bitmask){
  clearBits(GPINTENY, bitmask);
}

void mPortZIntDisable(unsigned char bitmask){
  clearBits(GPINTENZ, bitmask);
}

void mPortYEnablePullUp(unsigned char bitmask){
  setBits(GPPUY, bitmask);
}

void mPortZEnablePullUp(unsigned char bitmask){
  setBits(GPPUZ, bitmask);
}

void mPortYDisablePullUp(unsigned char bitmask){
  clearBits(GPPUY, bitmask);
}

void mPortZDisablePullUp(unsigned char bitmask){
  clearBits(GPPUZ, bitmask);
}

inline void writePE(unsigned char reg_addr, unsigned char data) {
  unsigned char junk = 0;
   
  // test for ready
  while (TxBufFullSPI1());
  
  // CS low to start transaction
  CLEAR_CS
  // 8-bits
  SPI_Mode8;
  // OPCODE and HW Address (Should always be 0b0100000), set LSB for write
  WriteSPI1((PE_OPCODE_HEADER | WRITE));
  // test for done
  while (SPI1STATbits.SPIBUSY); // wait for byte to be sent
  junk = ReadSPI1();
  // Input Register Address

  WriteSPI1(reg_addr);
  while (SPI1STATbits.SPIBUSY); // wait for byte to be sent
  junk = ReadSPI1();
  // One byte of data to write to register
  
  WriteSPI1(data);
  // test for done
  while (SPI1STATbits.SPIBUSY); // wait for end of transaction
  junk = ReadSPI1();
  // CS high
  SET_CS
  
}

inline unsigned char readPE(unsigned char reg_addr) {
  unsigned char out = 0;
  
  // test for ready
  while (TxBufFullSPI1());
  
  // CS low to start transaction
  CLEAR_CS
  
  // 8-bits
  SPI_Mode8;
  // OPCODE and HW Address (Should always be 0b0100000), clear LSB for write
  WriteSPI1((PE_OPCODE_HEADER | READ));
  // test for done
  while (SPI1STATbits.SPIBUSY); // wait for byte to be sent
  out = ReadSPI1(); //junk
  // Input Register Address
  // test for ready
  while (TxBufFullSPI1());
  // 8-bits
  
  WriteSPI1(reg_addr);
  while (SPI1STATbits.SPIBUSY); // wait for byte to be sent
  out = ReadSPI1(); // junk
  // One byte of dummy data to write to register
  // test for ready
  while (TxBufFullSPI1());
  // 8-bits
  
  WriteSPI1(out);
  // test for done
  while (SPI1STATbits.SPIBUSY); // wait for end of transaction
  out = ReadSPI1(); // bingo
  // CS high
  SET_CS
  
  return out;
}
