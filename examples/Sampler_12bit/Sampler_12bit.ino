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
  A 12-bit sampler to record sampled audio to SRAM.
  Input is sampled at 44.1 kHz and reproduced on the output.
  Recordings sampled at 22 kHz and stored to SRAM.

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

unsigned int playbackBuf = 2048;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int playbackSampleRate;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
volatile boolean warning = false;
boolean adjustablePlaybackSpeed = false;  // set to true with pot connected to A0
int currentA0Position;
volatile long address = 0;
volatile long endAddress = 0;
volatile byte addressChipNumber = 0;
volatile byte endAddressChipNumber = 0;

volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
unsigned int readBuf[2];
unsigned int writeBuf;
boolean evenCycle = true;
unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded = false;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  recordingSampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  passthroughSampleRate = DEFAULT_SAMPLE_RATE;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(PLAY_BUTTON, INPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(PLAY_BUTTON, HIGH);

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

    // Print the number of instruction cycles remaining at the end of the ISR.
    // The more work you try to do in the ISR, the lower this number will become.
    // If the number of cycles remaining reaches 0, then the ISR will take up
    // all the CPU time and the code in loop() will not run.

    Serial.print("even cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndEven);
    Serial.print("   odd cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndOdd);
    Serial.println();
    if (((UINT16_MAX - timer1EndEven) < 20) || (((UINT16_MAX - timer1EndOdd) < 20))) {
      Serial.println("WARNING: ISR execution time is too long. Reduce sample rate or reduce the amount of code in the ISR.");
    }

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
      Serial.print(recordEndTime - recordStartTime);
      Serial.println(" ms");
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
      Serial.print(millis() - recordStartTime);
      Serial.println(" ms");
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
      if (adjustablePlaybackSpeed) {
        playbackSampleRate = map(currentA0Position-analogRead(0), -1023, 1023, recordingSampleRate+20000, recordingSampleRate-20000);
      } else {
        playbackSampleRate = recordingSampleRate;
      }
      // compute the start value for counter1 to achieve the chosen playback rate
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);
    }
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  unsigned int signal;

  if (mode != RECORD_DONE) {
    AudioHacker.writeDAC(playbackBuf);
  }


  if ((mode != PLAYBACK) && (mode != RECORD_DONE)) {
    // Read ADC
    signal = AudioHacker.readADC();
  }

  if (mode == RECORD) {
    if (evenCycle) {
      // we only write to memory on odd cycles, so buffer the sampled signal.
      writeBuf = signal;
    } else {
      // Write two samples to SRAM
      AudioHacker.writeSRAMPacked(addressChipNumber, address, writeBuf, signal);

      address += 3;
      if (address > MAX_ADDR) {
        if (addressChipNumber == 0) {
          // proceed to the second SRAM chip
          address = 0;
          addressChipNumber = 1;
        } else {
          // end of memory, stop recording
          mode = RECORD_DONE;
          endAddress = address;
          endAddressChipNumber = 1;
          address = 0; // loop around to beginning of memory
          addressChipNumber = 0;
        }
      }
    }
  }

  if (mode == PLAYBACK) {
    if (evenCycle) {
      // Read from SRAM. Two 12-bit samples are read into readBuf[0] and readBuf[1].
      AudioHacker.readSRAMPacked(addressChipNumber, address, readBuf);
      signal = readBuf[0];

      address += 3;
      if (address > MAX_ADDR) {
        if (addressChipNumber == 0) {
          address = 0;
          addressChipNumber = 1;
        } else {
          address = 0;
          addressChipNumber = 0;
        }
      }
      if ((address == endAddress) && (addressChipNumber == endAddressChipNumber)) {
        address = 0;
        addressChipNumber = 0;
      }
    } else {
      signal = readBuf[1];
    }
  } // PLAYBACK

  playbackBuf = signal;

#ifdef DEBUG
  if (evenCycle) {
    timer1EndEven = TCNT1;
  } else {
    timer1EndOdd = TCNT1;
  }
#endif
  evenCycle = !evenCycle;
}


