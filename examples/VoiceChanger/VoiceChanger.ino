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
  8-bit Voice Changer for Audio Hacker.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */

#include <AudioHacker.h>

//#define DEBUG
#define OFF 0
#define PASSTHROUGH 1
#define RECORD 2
#define PLAYBACK 3
#define RECORD_DONE 4
#define RECORD_BUTTON 5
#define PLAY_BUTTON 6

unsigned int playbackBuf = 128;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int playbackSampleRate;
unsigned int nSamplesToPlay = 128;;
volatile unsigned int nSamplesPlayed = 0;
unsigned int grainSize = 128;
byte stretch = 1;
unsigned int timer1Start;
volatile unsigned int timer1End;
unsigned long lastDebugPrint = 0;
int currentA0Position;
volatile long address = 0;
volatile byte addressChipNumber = 0;
volatile long endAddress = 0;
volatile byte endAddressChipNumber = 0;
volatile long grainAddress = 0;
volatile byte grainAddressChipNumber = 0;

volatile byte mode = PASSTHROUGH;
unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded = false;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  recordingSampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  passthroughSampleRate = DEFAULT_SAMPLE_RATE_8BIT;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(PLAY_BUTTON, INPUT);
  pinMode(2, INPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(PLAY_BUTTON, HIGH);
  digitalWrite(2, HIGH);

  AudioHacker.begin();

#ifdef DEBUG
  Serial.print("sample rate = ");
  Serial.print(passthroughSampleRate);
  Serial.print(" Hz, recording sample rate = ");
  Serial.print(recordingSampleRate);
  Serial.print(" Hz");
  Serial.println();
#endif
}

void loop() {

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();

    Serial.print("cycles remaining = ");
    Serial.print(UINT16_MAX - timer1End);
    Serial.print("   playbackRate = ");
    Serial.print(playbackSampleRate);
    Serial.print("   grainSize = ");
    Serial.print(grainSize);
    Serial.print("   stretch = ");
    Serial.print(stretch);

    Serial.println();
  }
#endif

  if ((mode == OFF) || (mode == PASSTHROUGH)) {
    if (digitalRead(RECORD_BUTTON) == LOW) {
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
        // debounce the record button.
        recordStartTime = 0;
        return;
      }
      mode = RECORD;
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
      currentA0Position = analogRead(0);
      address = 0;
      addressChipNumber = 0;
    }
    if ((digitalRead(PLAY_BUTTON) == LOW) && (sampleRecorded)) {
      mode = PLAYBACK;
      address = 0;
      addressChipNumber = 0;
      grainAddress = 0;
      grainAddressChipNumber = 0;
      nSamplesPlayed = 0;
    }
  }

  if (mode == PASSTHROUGH) {
    timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
  }

  if (mode == RECORD) {
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) {
        // debounce
        return;
      }
#ifdef DEBUG
      Serial.print("recording time = ");
      Serial.println(recordEndTime - recordStartTime);
#endif
      sampleRecorded = true;
      endAddress = address;
      endAddressChipNumber = addressChipNumber;
      mode = PASSTHROUGH;
      address = 0;
      addressChipNumber = 0;
    }
  }

  if (mode == RECORD_DONE) {
    if (recordStartTime != 0) {
#ifdef DEBUG
      Serial.print("recording time = ");
      Serial.println(millis() - recordStartTime);
#endif
      sampleRecorded = true;
      recordStartTime = 0;
    }
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      mode = PASSTHROUGH;
    }
  }

  if (mode == PLAYBACK) {
    if (digitalRead(PLAY_BUTTON) == HIGH) {
      // play button released
      mode = PASSTHROUGH;
      address = 0;
      addressChipNumber = 0;
    } else {
      grainSize = map(analogRead(1), 0, 1023, 2, 2048);
      stretch = map(analogRead(2), 0, 1023, 1, 10);
      playbackSampleRate = map(currentA0Position-analogRead(0), -1023, 1023, recordingSampleRate+20000, recordingSampleRate-20000);
      // nSamplesToPlay is the number of samples to play from each grain.
      nSamplesToPlay = stretch * grainSize * ((float)playbackSampleRate/(float)recordingSampleRate);
      // compute the start value for counter1 to achieve the chosen playback rate
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);
    }
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  byte signal;

  if (mode != RECORD_DONE) {
    AudioHacker.writeDAC_8bit(playbackBuf);
  }


  if ((mode != PLAYBACK) && (mode != RECORD_DONE)) {
    // Read ADC
    signal = AudioHacker.readADC_8bit();
  }

  if (mode == RECORD) {
    AudioHacker.writeSRAM(addressChipNumber, address, signal);

    address++;
    if (address > 131071) {
      // end of memory, stop recording
      mode = RECORD_DONE;
      endAddress = address;
      endAddressChipNumber = 0;
      address = 0; // loop around to beginning of memory
      addressChipNumber = 0;
    }
  }


  if (mode == PLAYBACK) {
    signal = AudioHacker.readSRAM(addressChipNumber, address);

    nSamplesPlayed++;
    if (nSamplesPlayed >= nSamplesToPlay) {
      // proceed to the next grain
      nSamplesPlayed = 0;
      grainAddress += grainSize;
      if ((grainAddress > endAddress) || (grainAddress > 131071)) {
        grainAddress = 0;
      }
      address = grainAddress;
      playbackBuf = signal;
      return;
    }

    address++;
    if (address > 131071) {
      address = 0;
      addressChipNumber = 0;
    }
    if ((address == endAddress) && (addressChipNumber == endAddressChipNumber)) {
      address = 0;
      addressChipNumber = 0;
    }
    if (address == (grainAddress + grainSize)) {
      address = grainAddress; // start over within the grain
    }
  } // PLAYBACK

  playbackBuf = signal;

#ifdef DEBUG
  timer1End = TCNT1;
#endif
}


