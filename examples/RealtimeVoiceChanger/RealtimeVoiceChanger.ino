/*
  Audio Hacker Library
  Copyright (C) 2017 nootropic design, LLC
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
  8-bit Realtime Voice Changer for Audio Hacker.

 */

#include <AudioHacker.h>

//#define DEBUG
#define GRAINSIZE 512

volatile unsigned int output = 2048;
unsigned int timer1Start;
volatile unsigned int timer1End;
volatile unsigned int counter = 0;
byte counterMod = 2;
unsigned long lastDebugPrint = 0;
boolean normal = false;
volatile long readAddress = 0;
volatile long writeAddress = 0;
unsigned int buf[GRAINSIZE];


void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  timer1Start = UINT16_MAX - (F_CPU / DEFAULT_RECORDING_SAMPLE_RATE);
  AudioHacker.begin();
}

void loop() {

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();
    Serial.print("t1=");
    Serial.print(UINT16_MAX - timer1End);
    Serial.println();
  }
#endif

  counterMod = map(analogRead(0), 0, 1024, 2, 11);
  normal = (counterMod == 10);
}


ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  unsigned int signal;
  boolean updateTime = false;

  AudioHacker.writeDAC(output);

  signal = AudioHacker.readADC();
  buf[writeAddress] = signal;

  counter++;
  if (((counter % counterMod) != 0) || (normal)) {
    output = buf[readAddress];
    readAddress++;

    // cross-fade output if we are approaching the end of the grain
    // mix the output with the current realtime signal
    unsigned int distance = GRAINSIZE - writeAddress;
    if (distance <= 16) {
      // weighted average of output and current input yields 16 bit number
      unsigned int s = (output * distance) + (signal * (16-distance)); 
      s = s >> 4; // reduce to 12 bits.
      output = s;
    }
  }

  writeAddress++;
  if (writeAddress >= GRAINSIZE) {
    writeAddress = 0; // loop around to beginning of grain
    readAddress = 0;
  }

#ifdef DEBUG
  timer1End = TCNT1;
#endif
}


