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
  The SRAM is divided into 4 equal parts so that 4 samples can be
  recorded.

  This sketch is designed for use with the DJ Shield but can be used with
  buttons on a breadboard:
  Record button = D5
  Sample 0 button = D6
  Sample 1 button = D4
  Sample 2 button = D3
  Sample 3 button = D2

  To record a sample, press and hold the record button, then hold a sample button
  for the recording duration.
  To play a sample, press and hold the corresponding sample button.

  Input is sampled at 44.1 kHz and reproduced on the output.
  Recordings sampled at 22 kHz and stored to SRAM.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */

  

#include <EEPROM.h>
#include <AudioHacker.h>

#define DEBUG
#define OFF 0
#define PASSTHROUGH 1
#define RECORD 2
#define PLAYBACK 3
#define RECORD_DONE 4
#define RECORD_BUTTON 5
#define SAMPLE0_BUTTON 6
#define SAMPLE1_BUTTON 4
#define SAMPLE2_BUTTON 3
#define SAMPLE3_BUTTON 2

unsigned int playbackBuf = 2048;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int timer1Start;
volatile unsigned int timer1EndEven;
volatile unsigned int timer1EndOdd;
volatile long address = 0;
volatile long endAddress[4];
volatile byte addressChipNumber = 0;

volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
unsigned int readBuf[2];
unsigned int writeBuf;
boolean evenCycle = true;

// set to true if you are using a battery backup on the
// SRAM and want to keep sample address info in EEPROM
boolean batteryBackup = true; 

unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded[4];
byte sample;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  recordingSampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  passthroughSampleRate = DEFAULT_SAMPLE_RATE;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(SAMPLE0_BUTTON, INPUT);
  pinMode(SAMPLE1_BUTTON, INPUT);
  pinMode(SAMPLE2_BUTTON, INPUT);
  pinMode(SAMPLE3_BUTTON, INPUT);
  
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(SAMPLE0_BUTTON, HIGH);
  digitalWrite(SAMPLE1_BUTTON, HIGH);
  digitalWrite(SAMPLE2_BUTTON, HIGH);
  digitalWrite(SAMPLE3_BUTTON, HIGH);

  // DJ Shield LEDs
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);

  AudioHacker.begin();

#ifdef DEBUG
  Serial.print("sample rate = ");
  Serial.print(passthroughSampleRate);
  Serial.print(" Hz, recording sample rate = ");
  Serial.print(recordingSampleRate);
  Serial.print(" Hz");
  Serial.println();
#endif

  sampleRecorded[0] = false;
  sampleRecorded[1] = false;
  sampleRecorded[2] = false;
  sampleRecorded[3] = false;

  if (batteryBackup) {
    // Read endAddress[] values from EEPROM when we have a battery
    // connected to the Audio Hacker to preserve SRAM contents.
    for (byte i=0;i<4;i++) {
      byte a = i*3;
      long b;
      b = (long)EEPROM.read(a);
      endAddress[i] = (b << 16);
      b = (long)EEPROM.read(a+1);
      endAddress[i] |= (b << 8);
      b = (long)EEPROM.read(a+2);
      endAddress[i] |= b;

      if (endAddress[i] > 0) {
	sampleRecorded[i] = true;
#ifdef DEBUG
	Serial.print("sample ");
	Serial.print(i);
	Serial.print(" endAddress = ");
	Serial.print(endAddress[i]);
	Serial.println();
#endif
      }
    }
  }
}

void loop() {


#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();

    // Print the number of instruction cycles remaining at the end of the ISR.
    // The more work you try to do in the ISR, the lower this number will become.
    // If the number of cycles remaining reaches 0, then the ISR will take up
    // all the CPU time and the code in loop() will not run.

    /*
    Serial.print("even cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndEven);
    Serial.print("   odd cycles remaining = ");
    Serial.print(UINT16_MAX - timer1EndOdd);
    Serial.println();
    if (((UINT16_MAX - timer1EndEven) < 20) || (((UINT16_MAX - timer1EndOdd) < 20))) {
      Serial.println("WARNING: ISR execution time is too long. Reduce sample rate or reduce the amount of code in the ISR.");
    }
    */
  }
#endif

  if ((mode == OFF) || (mode == PASSTHROUGH)) {
    if ((digitalRead(RECORD_BUTTON) == LOW) && ((digitalRead(SAMPLE0_BUTTON) == LOW) || (digitalRead(SAMPLE1_BUTTON) == LOW) || (digitalRead(SAMPLE2_BUTTON) == LOW) || (digitalRead(SAMPLE3_BUTTON) == LOW))) {
      // enter RECORD mode
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
	// debounce the record button.
	recordStartTime = 0;
	return;
      }
      if (digitalRead(SAMPLE0_BUTTON) == LOW) {
	sample = 0;
	address = 0;
	addressChipNumber = 0;
      }
      if (digitalRead(SAMPLE1_BUTTON) == LOW) {
	sample = 1;
	address = 65535;
	addressChipNumber = 0;
      }
      if (digitalRead(SAMPLE2_BUTTON) == LOW) {
	sample = 2;
	address = 0;
	addressChipNumber = 1;
      }
      if (digitalRead(SAMPLE3_BUTTON) == LOW) {
	sample = 3;
	address = 65535;
	addressChipNumber = 1;
      }
      mode = RECORD;
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    } else {
      // enter PLAYBACK mode
      if ((digitalRead(SAMPLE0_BUTTON) == LOW) && (sampleRecorded[0])) {
	address = 0;
	addressChipNumber = 0;
	sample = 0;
	mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE1_BUTTON) == LOW) && (sampleRecorded[1])) {
	address = 65535;
	addressChipNumber = 0;
	sample = 1;
	mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE2_BUTTON) == LOW) && (sampleRecorded[2])) {
	address = 0;
	addressChipNumber = 1;
	sample = 2;
	mode = PLAYBACK;
      }
      if ((digitalRead(SAMPLE3_BUTTON) == LOW) && (sampleRecorded[3])) {
	address = 65535;
	addressChipNumber = 1;
	sample = 3;
	mode = PLAYBACK;
      }
    }
  }

  if (mode == PASSTHROUGH) {
    timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
  }

  if (mode == RECORD) {
    digitalWrite(A4, HIGH);
  
    if (((sample == 0) && (digitalRead(SAMPLE0_BUTTON) == HIGH)) || 
	((sample == 1) && (digitalRead(SAMPLE1_BUTTON) == HIGH)) || 
	((sample == 2) && (digitalRead(SAMPLE2_BUTTON) == HIGH)) || 
	((sample == 3) && (digitalRead(SAMPLE3_BUTTON) == HIGH))) {
      // recording stopped
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) {
	// debounce
	return;
      }
      sampleRecorded[sample] = true;
      endAddress[sample] = address;
#ifdef DEBUG
      Serial.print("sample ");
      Serial.print(sample);
      Serial.print(" recording time = ");
      Serial.print(recordEndTime - recordStartTime);
      Serial.println(" ms");
      Serial.print(" endAddress = ");
      Serial.println(endAddress[sample]);
#endif

      if (batteryBackup) {
	// Write endAddress to EEPROM for battery backup use.
	byte a = sample*3;
	EEPROM.write(a, (endAddress[sample] >> 16) & 0xFF);
	EEPROM.write(a+1, (endAddress[sample] >> 8) & 0xFF);
	EEPROM.write(a+2, endAddress[sample] & 0xFF);
      }
      mode = PASSTHROUGH;
    }
  } else {
    digitalWrite(A4, LOW);
  }


  if (mode == RECORD_DONE) {
    if (recordStartTime != 0) {
#ifdef DEBUG
      Serial.print("sample ");
      Serial.print(sample);
      Serial.print(" recording time = ");
      Serial.print(millis() - recordStartTime);
      Serial.println(" ms");
      Serial.print(" endAddress = ");
      Serial.println(endAddress[sample]);
#endif
      sampleRecorded[sample] = true;
      recordStartTime = 0;

      if (batteryBackup) {
	// Write endAddress to EEPROM for battery backup use.
	byte a = sample*3;
	EEPROM.write(a, (endAddress[sample] >> 16) & 0xFF);
	EEPROM.write(a+1, (endAddress[sample] >> 8) & 0xFF);
	EEPROM.write(a+2, endAddress[sample] & 0xFF);
      }
    }
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      mode = PASSTHROUGH;
    }
  }

  if (mode == PLAYBACK) {
    digitalWrite(A5, HIGH);
    if (((sample == 0) && (digitalRead(SAMPLE0_BUTTON) == HIGH)) || 
	((sample == 1) && (digitalRead(SAMPLE1_BUTTON) == HIGH)) || 
	((sample == 2) && (digitalRead(SAMPLE2_BUTTON) == HIGH)) || 
	((sample == 3) && (digitalRead(SAMPLE3_BUTTON) == HIGH))) {
      // play button released
      mode = PASSTHROUGH;
    } else {
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
  } else {
    digitalWrite(A5, LOW);
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
      // Write to SRAM
      AudioHacker.writeSRAMPacked(addressChipNumber, address, writeBuf, signal);

      address += 3;
      if (((sample == 0) && (address > 65532)) || 
	  ((sample == 1) && (address > MAX_ADDR)) || 
	  ((sample == 2) && (address > 65532)) || 
	  ((sample == 3) && (address > MAX_ADDR))) {
	// end of memory, stop recording
	mode = RECORD_DONE;
	endAddress[sample] = address;
      }
    }
  }


  if (mode == PLAYBACK) {
    if (evenCycle) {
      // Read from SRAM
      AudioHacker.readSRAMPacked(addressChipNumber, address, readBuf);
      signal = readBuf[0];

      address += 3;
      if ((sample == 0) && (address == endAddress[0])) {
	address = 0;
	addressChipNumber = 0;
      }
      if ((sample == 1) && (address == endAddress[1])) {
	address = 65535;
	addressChipNumber = 0;
      }
      if ((sample == 2) && (address == endAddress[2])) {
	address = 0;
	addressChipNumber = 1;
      }
      if ((sample == 3) && (address == endAddress[3])) {
	address = 65535;
	addressChipNumber = 1;
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


