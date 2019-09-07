/*
  Audio Hacker Library
  Copyright (C) 2019 nootropic design, LLC
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
/*
  Realtime reverse effect for Audio Hacker.

  See Audio Hacker project page for details.
  https://nootropicdesign.com/audio-hacker/projects/
 */

#include <AudioHacker.h>

#define HALF_MEMORY 65534

unsigned int output;
unsigned int sampleRate;
unsigned int timer1Start;
volatile long writeAddress = 0;
volatile long readAddress = HALF_MEMORY;
volatile boolean bufferFilled = false;

void setup() {
  output = 2048; // silence
  sampleRate = 18000;
  timer1Start = UINT16_MAX - (F_CPU / sampleRate);

  AudioHacker.begin();
}

void loop() {
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  unsigned int input;

  AudioHacker.writeDAC(output);

  // Read ADC
  input = AudioHacker.readADC();

  if (bufferFilled) {
    AudioHacker.readSRAM(0, readAddress, (byte *)&output, 2);
    readAddress -= 2; // each sample uses 2 bytes. Read backwards in memory.
    if (readAddress < 0) {
      readAddress = MAX_ADDR-1;
    }
  }

  AudioHacker.writeSRAM(0, writeAddress, (byte *)&input, 2);
  writeAddress += 2; // each sample uses 2 bytes. Write forward in memory.
  if (writeAddress > HALF_MEMORY) {
    bufferFilled = true;
  }
  if (writeAddress > MAX_ADDR) {
    writeAddress = 0;
  }
}
