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
/*
  Echo Effect for Audio Hacker.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */

#include <AudioHacker.h>

#define DEBUG

unsigned int playbackBuf;
unsigned int sampleRate;
unsigned int readBuf[2];
unsigned int writeBuf;
boolean evenCycle = true;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
unsigned long lastDebugPrint = 0;
volatile long address = 0;
unsigned int echoDelay;
boolean echoWrapped = false;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  playbackBuf = 2048;
  sampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  timer1Start = UINT16_MAX - (F_CPU / sampleRate);

  AudioHacker.begin();

#ifdef DEBUG
  Serial.print("sample rate = ");
  Serial.print(sampleRate);
  Serial.print(" Hz");
  Serial.println();
#endif
}

void loop() {

  delay(300);
  // echoDelay is number of memory slots back into the past to read for the echo.
  // must be a factor of 3 because we pack 2 samples into each 3-byte sequence of SRAM.
  echoDelay = analogRead(0) * 30; 

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();

    //each 3 echoDelay2 is 2 samples.  
    unsigned int delayMillis = (float)(((float)((echoDelay * 2) / 3)) / (float)(sampleRate/1000.0));

    Serial.print("echo delay = ");
    Serial.print(delayMillis);
    Serial.print(" ms    even cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndEven);
    Serial.print("   odd cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndOdd);
    Serial.println();
  }
}
#endif

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  unsigned int signal;
  unsigned int echo;
  int mix;

  AudioHacker.writeDAC(playbackBuf);

  // Read ADC
  signal = AudioHacker.readADC();


  if (evenCycle) {
    long echoAddress = address - echoDelay;
    if (echoAddress < 0) {
      echoAddress += MAX_ADDR;
    }
    AudioHacker.readSRAMPacked(0, echoAddress, readBuf);
    if ((!echoWrapped) && (echoAddress > address)) {
      // avoid reading from unwritten memory
      echo = 2048;
      readBuf[1] = 2048;
    } else {
      echo = readBuf[0];
    }
  } else {
    echo = readBuf[1];
  }
  if (echoDelay == 0) {
    echo = 2048;
  }

  if (evenCycle) {
    writeBuf = signal;
  } else {
    AudioHacker.writeSRAMPacked(0, address, writeBuf, signal);
    address += 3;
    if (address > MAX_ADDR) {
      address = 0;
      echoWrapped = true;
    }
  }


  mix = signal-2048;
  echo = echo >> 1; // attenuate echo
  mix += (echo - 1024); // since we are dividing echo by 2, decrement by 1024
  if (mix < -2048) {
    mix = -2048;
  } else {
    if (mix > 2047) {
      mix = 2047;
    }
  }
  playbackBuf = mix + 2048;


#ifdef DEBUG
  if (evenCycle) {
    timer1EndEven = TCNT1;
  } else {
    timer1EndOdd = TCNT1;
  }
#endif
  evenCycle = !evenCycle;
}


