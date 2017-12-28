/*
  Audio Hacker Library
  Copyright (C) 2013 nootropic design, LLC
  All rights reserved.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include "AudioHacker.h"

AudioHackerClass::AudioHackerClass() {
}



void AudioHackerClass::begin() {
  pinMode(SCK, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SS, OUTPUT);

  digitalWrite(SCK, LOW);
  digitalWrite(MISO, LOW);
  digitalWrite(MOSI, LOW);
  digitalWrite(SS, HIGH);

  pinMode(SRAM0_SS, OUTPUT);
  pinMode(SRAM1_SS, OUTPUT);
  pinMode(DAC_SS, OUTPUT);
  pinMode(ADC_SS, OUTPUT);

  digitalWrite(SRAM0_SS, HIGH);
  digitalWrite(SRAM1_SS, HIGH);
  digitalWrite(DAC_SS, HIGH);
  digitalWrite(ADC_SS, HIGH);

  // Configure SPI
  SPCR |= _BV(SPE);    // enable SPI
  SPCR &= ~_BV(SPIE);  // SPI interrupts off
  SPCR &= ~_BV(DORD);  // MSB first
  SPCR |= _BV(MSTR);   // SPI master mode
  SPCR &= ~_BV(CPOL);  // leading edge rising
  SPCR &= ~_BV(CPHA);  // sample on leading edge
  SPCR &= ~_BV(SPR1);  // speed = clock/4
  SPCR &= ~_BV(SPR0);  
  SPSR |= _BV(SPI2X);  // 2X speed

  // Disable Timer1 interrupt
  TIMSK1 &= ~_BV(TOIE1);

  TCCR1A = 0;
  TCCR1B = 0;

  // Set prescalar to 1;
  TCCR1B |= _BV(CS10);

  // Enable Timer1 interrupt
  TIMSK1 |= _BV(TOIE1);
}

//  old version that used hardware SPI at 1MHz
/*
unsigned int AudioHackerClass::readADC() {
  unsigned int signal;
  byte highByte;
  byte lowByte;
  byte tmp;

  SPCR |= _BV(SPR0);    // set SPI clock to clock/16 = 1MHz
  SPSR &= ~_BV(SPI2X);
  SELECT_ADC;
  highByte = spiRead();
  tmp = (highByte & 0x01) << 7;
  lowByte = spiRead();
  lowByte = (lowByte >> 1) | tmp;
  DESELECT_ADC;
  signal = highByte & 0x1E;
  signal = (signal << 7) | lowByte;
  SPCR &= ~_BV(SPR0);  // restore clock to clock/2 = 8MHz
  SPSR |= _BV(SPI2X);
  return signal;
}
*/

/*
 * The MCP3201 ADC has a maximum SPI clock speed of 1.6MHz.
 * Instead of using hardware SPI at 1MHz, we bit-bang the SPI
 * on the normal SPI pins at approximately 1.6MHz.
 * Adapted from McpSarAdc library under GPL version 3.
 */
unsigned int AudioHackerClass::readADC() {
  unsigned int signal = 0;

  SPCR &= ~_BV(SPE);    // disable hardware SPI while we bang this out

  SELECT_ADC;

  // begin sample
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_HIGH;
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_LOW;

  // second clock
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_HIGH;
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_LOW;

  // null bit
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_HIGH;
  NOP;
  NOP;
  NOP;
  NOP;
  SCK_LOW;

  readBit(signal, 11);
  readBit(signal, 10);
  readBit(signal,  9);
  readBit(signal,  8);
  readBit(signal,  7);
  readBit(signal,  6);
  readBit(signal,  5);
  readBit(signal,  4);
  readBit(signal,  3);
  readBit(signal,  2);
  readBit(signal,  1);
  readBit(signal,  0);

  DESELECT_ADC;

  SPCR |= _BV(SPE);    // enable hardware SPI

  return signal;
}

byte AudioHackerClass::readADC_8bit() {
  return (readADC() >> 4);
}


void AudioHackerClass::writeDAC(unsigned int signal) {
  SELECT_DAC;
  spiWrite(DAC_CONFIG | ((signal >> 8) & 0xF)); // 4 config bits and bits 11-8
  spiWrite(signal); // bits 7-0
  DESELECT_DAC;
}

void AudioHackerClass::writeDAC_8bit(byte signal) {
  SELECT_DAC;
  spiWrite(DAC_CONFIG | ((signal >> 4) & 0xF)); // 4 config bits and bits 7-4
  spiWrite(signal << 4); // bits 3-0 of 8 bit signal.  Least significant 4 bits are 0
  DESELECT_DAC;
}


void AudioHackerClass::writeSRAM(byte chipNumber, long address, byte data) {
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x02);           // mode = write
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  spiWrite(data);
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}

void AudioHackerClass::writeSRAM(byte chipNumber, long address, byte *buf, int len) {
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x02);           // mode = write
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  for(int i=0;i<len;i++) {
    spiWrite(*buf);
    buf++;
  }
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}

void AudioHackerClass::writeSRAM(byte chipNumber, long address, unsigned int data) {
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x02);           // mode = write
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  spiWrite(data>>8);
  spiWrite(data);
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}

// Writes two ints (4 bytes) representing 2 12-bit samples in the space of 3 bytes.
void AudioHackerClass::writeSRAMPacked(byte chipNumber, long address, unsigned int data1, unsigned int data2) {
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x02);           // mode = write
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  data1 = data1<<4;
  spiWrite(data1 >> 8);
  spiWrite(data1 | (data2 >> 8));
  spiWrite(data2);
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}


byte AudioHackerClass::readSRAM(byte chipNumber, long address) {
  byte data;
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x03);           // mode = read
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  data = spiRead();
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
  return data;
}

void AudioHackerClass::readSRAM(byte chipNumber, long address, byte *buf, int len) {
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x03);           // mode = read
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  for(int i=0;i<len;i++) {
    *buf = spiRead();
    buf++;
  }
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}

// Reads 3 bytes that represent 2 12-bit samples
void AudioHackerClass::readSRAMPacked(byte chipNumber, long address, unsigned int *buf) {
  byte highByte;
  byte lowByte;
  if (chipNumber == 0) {
    SELECT_SRAM0;
  } else {
    SELECT_SRAM1;
  }
  spiWrite(0x03);           // read mode
  spiWrite(address >> 16);  // address
  spiWrite(address >> 8);
  spiWrite(address);
  highByte = spiRead();
  lowByte = spiRead();
  *buf = highByte;
  *buf = (*buf << 4) | (lowByte >> 4);
  highByte = lowByte;

  lowByte = spiRead();

  buf++;
  *buf = highByte & 0x0F;
  *buf = (*buf << 8) | lowByte;
  if (chipNumber == 0) {
    DESELECT_SRAM0;
  } else {
    DESELECT_SRAM1;
  }
}

AudioHackerClass AudioHacker = AudioHackerClass();
