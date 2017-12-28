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

#ifndef AudioHacker_h
#define AudioHacker_h

#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include <pins_arduino.h>
#endif


#define UINT16_MAX 65535

#define SRAM0_SS 10
#define SRAM1_SS 9
#define DAC_SS 8
#define ADC_SS 7

// Uno, Duemilanove, Diecimila
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) 
// PB5 = D13
#define SCK_LOW PORTB &= ~0x20
#define SCK_HIGH PORTB |= 0x20
// PB4 = D12
#define MISO_READ PINB & 0x10
// PB2 = D10
#define SELECT_SRAM0 PORTB &= ~0x04
#define DESELECT_SRAM0 PORTB |= 0x04
// PB1 = D9
#define SELECT_SRAM1 PORTB &= ~0x02
#define DESELECT_SRAM1 PORTB |= 0x02
// PB0 = D8
#define SELECT_DAC PORTB &= ~0x01
#define DESELECT_DAC PORTB |= 0x01
// PD7 = D7
#define SELECT_ADC PORTD &= ~0x80
#define DESELECT_ADC PORTD |= 0x80
#endif

// Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// PB1 = D52
#define SCK_LOW PORTB &= ~0x02
#define SCK_HIGH PORTB |= 0x02
// PB3 = D50
#define MISO_READ PINB & 0x08
// PB4 = D10
#define SELECT_SRAM0 PORTB &= ~0x10
#define DESELECT_SRAM0 PORTB |= 0x10
// PH6 = D9
#define SELECT_SRAM1 PORTH &= ~0x40
#define DESELECT_SRAM1 PORTH |= 0x40
// PH5 = D8
#define SELECT_DAC PORTH &= ~0x20
#define DESELECT_DAC PORTH |= 0x20
// PH4 = D7
#define SELECT_ADC PORTH &= ~0x10
#define DESELECT_ADC PORTH |= 0x10
#endif

// Leonardo
#if defined(__AVR_ATmega32U4__)
// PB1
#define SCK_LOW PORTB &= ~0x02
#define SCK_HIGH PORTB |= 0x02
// PB3
#define MISO_READ PINB & 0x08
// PB6 = D10
#define SELECT_SRAM0 PORTB &= ~0x40
#define DESELECT_SRAM0 PORTB |= 0x40
// PB5 = D9
#define SELECT_SRAM1 PORTB &= ~0x20
#define DESELECT_SRAM1 PORTB |= 0x20
// PB4 = D8
#define SELECT_DAC PORTB &= ~0x10
#define DESELECT_DAC PORTB |= 0x10
// PE6 = D7
#define SELECT_ADC PORTE &= ~0x40
#define DESELECT_ADC PORTE |= 0x40
#endif

#define DAC_CONFIG 0x30 // DACa, unbuffered, gain=1, not shutdown

#define MAX_ADDR 131067 // last address for 3-byte storage of 2 samples
// address range for 23LCV1024 chip is 0-131071.
// Since we pack two 12-bit samples in 3 bytes, we don't use the last 2 bytes of the chip.
// Addresses 0-131069 are used to store 43690 3-byte sets, each containing two 12-bit samples.
// So a 23LCV1024 chip can hold 87380 12-bit samples.
// For half the chip, we use Bytes 0-65534, so the MAX_ADDR for half the chip is 65532
// Second half uses 65535-131069

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// For some reason, the compiled ATMega code is less efficient so we need to go a bit slower.
#define DEFAULT_SAMPLE_RATE 42000
#define DEFAULT_SAMPLE_RATE_8BIT 40000
#else
#define DEFAULT_SAMPLE_RATE 44100
#define DEFAULT_SAMPLE_RATE_8BIT 42000
#endif

#define DEFAULT_RECORDING_SAMPLE_RATE 22050

#define NOP asm volatile ("nop\n\t")


class AudioHackerClass {
public:
  AudioHackerClass();
  void begin();
  unsigned int readADC();
  byte readADC_8bit();
  void writeSRAM(byte chipNumber, long address, byte data);
  void writeSRAM(byte chipNumber, long address, byte *buf, int len);
  void writeSRAM(byte chipNumber, long address, unsigned int data);
  void writeSRAMPacked(byte chipNumber, long address, unsigned int data1, unsigned int data2);
  byte readSRAM(byte chipNumber, long address);
  void readSRAM(byte chipNumber, long address, byte *buf, int len);
  void readSRAMPacked(byte chipNumber, long address, unsigned int *buf);
  void writeDAC(unsigned int signal);
  void writeDAC_8bit(byte signal);
};

static inline __attribute__((always_inline)) byte spiRead() {
  SPDR = 0xFF; // start SPI clock
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

static inline __attribute__((always_inline)) byte spiWrite(byte data) {
  SPDR = data;
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

static inline __attribute__((always_inline)) void readBit(uint16_t &v, uint8_t bit) {
  NOP;
  NOP;
  NOP;
  SCK_HIGH;
  NOP;
  NOP;
  if (MISO_READ) v |= (1 << bit);
  SCK_LOW;
}

extern AudioHackerClass AudioHacker;

#endif /* AudioHacker_h */

