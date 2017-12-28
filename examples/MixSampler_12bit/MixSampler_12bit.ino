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
#define RECORD2 3
#define PLAYBACK 4
#define PLAYBACK2 5
#define RECORD_DONE 6

#define RECORD_BUTTON 5
#define RECORD2_BUTTON 6
#define PLAY_BUTTON 4
#define PLAY2_BUTTON 3


unsigned int playbackBuf = 2048;
unsigned int sampleRate;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
volatile boolean warning = false;
volatile long address = 0;
volatile long endAddress = 0;
volatile long address2 = 0;
volatile long endAddress2 = 0;

volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
unsigned int readBuf[2];
unsigned int writeBuf;
boolean evenCycle = true;
unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded = false;
boolean sampleRecorded2 = false;
boolean attenuateSample = false;
boolean attenuateSignal = false;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  sampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  timer1Start = UINT16_MAX - (F_CPU / sampleRate);

  pinMode(RECORD_BUTTON, INPUT_PULLUP);
  pinMode(RECORD2_BUTTON, INPUT_PULLUP);
  pinMode(PLAY_BUTTON, INPUT_PULLUP);
  pinMode(PLAY2_BUTTON, INPUT_PULLUP);

  AudioHacker.begin();
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
      address = 0;
    }
    if (digitalRead(RECORD2_BUTTON) == LOW) {
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
        // debounce the record button.
        recordStartTime = 0;
        return;
      }
      mode = RECORD2;
      address = 0;
    }
    if ((digitalRead(PLAY_BUTTON) == LOW) && (sampleRecorded)) {
      mode = PLAYBACK;
      attenuateSample = (analogRead(0) < 350);
      attenuateSignal = (analogRead(0) > 700);
      address = 0;
    }
    if ((digitalRead(PLAY2_BUTTON) == LOW) && (sampleRecorded2)) {
      mode = PLAYBACK2;
      attenuateSample = (analogRead(1) < 350);
      attenuateSignal = (analogRead(1) > 700);
      address = 0;
    }
  }

  if (mode == RECORD) {
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) {
        // debounce
        return;
      }
      sampleRecorded = true;
      endAddress = address;
      mode = PASSTHROUGH;
      address = 0;
    }
  }
  if (mode == RECORD2) {
    if (digitalRead(RECORD2_BUTTON) == HIGH) {
      // record button released
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) {
        // debounce
        return;
      }
      sampleRecorded2 = true;
      endAddress2 = address;
      mode = PASSTHROUGH;
      address = 0;
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
  }

  if (mode == PLAYBACK) {
    if (digitalRead(PLAY_BUTTON) == HIGH) {
      // play button released
      mode = PASSTHROUGH;
      address = 0;
    }
  }
  if (mode == PLAYBACK2) {
    if (digitalRead(PLAY2_BUTTON) == HIGH) {
      // play button released
      mode = PASSTHROUGH;
      address = 0;
    }
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  unsigned int signal;
  unsigned int sample;
  int mix;


  AudioHacker.writeDAC(playbackBuf);
  signal = AudioHacker.readADC();
  playbackBuf = signal;

  if (mode == RECORD) {
    if (evenCycle) {
      // we only write to memory on odd cycles, so buffer the sampled signal.
      writeBuf = signal;
    } else {
      if (address <= MAX_ADDR) {
        // Write two samples to SRAM
        AudioHacker.writeSRAMPacked(0, address, writeBuf, signal);
        address += 3;
      }
    }
  }

  if (mode == RECORD2) {
    if (evenCycle) {
      // we only write to memory on odd cycles, so buffer the sampled signal.
      writeBuf = signal;
    } else {
      if (address <= MAX_ADDR) {
        // Write two samples to SRAM
        AudioHacker.writeSRAMPacked(1, address, writeBuf, signal);
        address += 3;
      }
    }
  }

  if (mode == PLAYBACK) {
    if (evenCycle) {
      // Read from SRAM. Two 12-bit samples are read into readBuf[0] and readBuf[1].
      AudioHacker.readSRAMPacked(0, address, readBuf);
      sample = readBuf[0];

      address += 3;
      if (address > MAX_ADDR) {
        address = 0;
      }
      if (address == endAddress) {
        address = 0;
      }
    } else {
      sample = readBuf[1];
    }

    // Mix sample onto signal

    if (attenuateSignal) {
      signal = signal >> 1;
      mix = signal-1024;
    } else {
      mix = signal-2048;
    }
    if (attenuateSample) {
      sample = sample >> 1;
      mix += (sample-1024);
    } else {
      mix += (sample-2048);
    }
    
    if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }
    playbackBuf = mix + 2048;
  } // PLAYBACK

  if (mode == PLAYBACK2) {
    if (evenCycle) {
      // Read from SRAM. Two 12-bit samples are read into readBuf[0] and readBuf[1].
      AudioHacker.readSRAMPacked(1, address, readBuf);
      sample = readBuf[0];

      address += 3;
      if (address > MAX_ADDR) {
        address = 0;
      }
      if (address == endAddress2) {
        address = 0;
      }
    } else {
      sample = readBuf[1];
    }

    // Mix sample onto signal

    if (attenuateSignal) {
      signal = signal >> 1;
      mix = signal-1024;
    } else {
      mix = signal-2048;
    }
    if (attenuateSample) {
      sample = sample >> 1;
      mix += (sample-1024);
    } else {
      mix += (sample-2048);
    }
    
    if (mix < -2048) {
      mix = -2048;
    } else {
      if (mix > 2047) {
        mix = 2047;
      }
    }
    playbackBuf = mix + 2048;
  } // PLAYBACK


#ifdef DEBUG
  if (evenCycle) {
    timer1EndEven = TCNT1;
  } else {
    timer1EndOdd = TCNT1;
  }
#endif
  evenCycle = !evenCycle;
}


