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
  Dynamically change the sample rate and bit resolution to hear how
  these affect the output.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */


#include <AudioHacker.h>

#define DEBUG
unsigned int playbackBuf = 2048;
unsigned int sampleRate = 22050;
byte resolution = 12;
unsigned int mask;
unsigned int timer1Start;
volatile unsigned int timer1End;

unsigned long lastDebugPrint = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  timer1Start = UINT16_MAX - (F_CPU / sampleRate);

  AudioHacker.begin();
}

void loop() {

  sampleRate = map(analogRead(0), 0, 1023, 1000, 44100);
  sampleRate = sampleRate - (sampleRate % 100); // round to nearest 100 Hz
  timer1Start = UINT16_MAX - (F_CPU / sampleRate);

  resolution = map(analogRead(1), 0, 1023, 1, 12);
  mask = 0x0FFF << (12-resolution);

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 500) {
    lastDebugPrint = millis();

    Serial.print("sample rate = ");
    Serial.print(sampleRate);
    Serial.print("   resolution = ");
    Serial.print(resolution);
    Serial.print(" bit");
    if (resolution > 1) Serial.print("s");
    Serial.println();
  }
#endif

  delay(300);

}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;

  AudioHacker.writeDAC(playbackBuf);


  playbackBuf = AudioHacker.readADC();
  playbackBuf &= mask;


#ifdef DEBUG
  timer1End = TCNT1;
#endif

}


