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
  3-Track Looper with Audio Mixing

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
*/

#include <EEPROM.h>
#include <AudioHacker.h>

#define DEBUG

#define PASSTHROUGH 0x01
#define RECORD 0x02
#define PLAY0 0x04
#define PLAY1 0x08
#define PLAY2 0x10
#define LOOP0 0x20
#define LOOP1 0x40
#define LOOP2 0x80

#define RECORD_BUTTON 5
#define LOOP_BUTTON 6
#define TRACK0_BUTTON 4
#define TRACK1_BUTTON 3
#define TRACK2_BUTTON 2

byte playbackBuf = 128;
unsigned int passthroughSampleRate;
unsigned int recordingSampleRate;
unsigned int timer1Start;
volatile unsigned int timer1End;
volatile long address[3];
volatile long endAddress[3];

// set to true if you are using a battery backup on the
// SRAM and want to keep sample address info in EEPROM
boolean batteryBackup = true; 

volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
boolean recordPressed;
unsigned int recordStartTime;
unsigned int recordEndTime;
volatile boolean trackRecorded[3];
volatile boolean exhausted = false;
byte track;
volatile unsigned int delayCounter[3];
unsigned int trackDelay[3];

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  trackRecorded[0] = false;
  trackRecorded[1] = false;
  trackRecorded[2] = false;

  recordingSampleRate = 18000;
  passthroughSampleRate = 32000;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(LOOP_BUTTON, INPUT);
  pinMode(TRACK0_BUTTON, INPUT);
  pinMode(TRACK1_BUTTON, INPUT);
  pinMode(TRACK2_BUTTON, INPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(LOOP_BUTTON, HIGH);
  digitalWrite(TRACK0_BUTTON, HIGH);
  digitalWrite(TRACK1_BUTTON, HIGH);
  digitalWrite(TRACK2_BUTTON, HIGH);
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

  if (batteryBackup) {
    // Read endAddress[] values from EEPROM when we have a battery
    // connected to the Audio Hacker to preserve SRAM contents.
    for (byte i=0;i<3;i++) {
      byte a = i*3;
      long b;
      b = (long)EEPROM.read(a);
      endAddress[i] = (b << 16);
      b = (long)EEPROM.read(a+1);
      endAddress[i] |= (b << 8);
      b = (long)EEPROM.read(a+2);
      endAddress[i] |= b;

      if (endAddress[i] > 0) {
	trackRecorded[i] = true;
#ifdef DEBUG
	Serial.print("track ");
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

  trackDelay[0] = map(analogRead(0), 0, 1023, 0, 65535);
  trackDelay[1] = map(analogRead(1), 0, 1023, 0, 65535);
  trackDelay[2] = map(analogRead(2), 0, 1023, 0, 65535);

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 1000) {
    lastDebugPrint = millis();

    if (mode & PASSTHROUGH) {
      Serial.print("PASSTHROUGH | ");
    } else {
      Serial.print("            | ");
    }
    if (mode & RECORD) {
      Serial.print("RECORD | ");
    } else {
      Serial.print("       | ");
    }
    if (mode & PLAY0) {
      Serial.print("PLAY0 | ");
    } else {
      Serial.print("      | ");
    }
    if (mode & PLAY1) {
      Serial.print("PLAY1 | ");
    } else {
      Serial.print("      | ");
    }
    if (mode & PLAY2) {
      Serial.print("PLAY2 | ");
    } else {
      Serial.print("      | ");
    }
    if (mode & LOOP0) {
      Serial.print("LOOP0 | ");
    } else {
      Serial.print("      | ");
    }
    if (mode & LOOP1) {
      Serial.print("LOOP1 | ");
    } else {
      Serial.print("      | ");
    }
    if (mode & LOOP2) {
      Serial.print("LOOP2");
    } else {
      Serial.print("     ");
    }
    Serial.print("   cycles = ");
    Serial.print(UINT16_MAX - timer1End);
    Serial.println();
  }
#endif

  if (digitalRead(RECORD_BUTTON) == LOW) {
    recordPressed = true;
  } else {
    if (recordPressed) {
      // record button released
      recordPressed = false;

      // toggle passthrough mode
      if (mode & PASSTHROUGH) {
	mode &= ~PASSTHROUGH;
      } else {
	timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
	mode |= PASSTHROUGH;
	mode &= ~(PLAY0 | PLAY1 | PLAY2 | LOOP0 | LOOP1 | LOOP2);
      }
    }
  }

  if (mode & PASSTHROUGH) {
    if ((recordPressed) && ((digitalRead(TRACK0_BUTTON) == LOW) || (digitalRead(TRACK1_BUTTON) == LOW) || (digitalRead(TRACK2_BUTTON) == LOW))) {
      // enter RECORD mode
      recordStartTime = millis();
      if ((recordStartTime - recordEndTime) < 20) {
	// debounce the record button.
	recordStartTime = 0;
	return;
      }
      if (digitalRead(TRACK0_BUTTON) == LOW) {
	track = 0;
	address[0] = 0;
      }
      if (digitalRead(TRACK1_BUTTON) == LOW) {
	track = 1;
	address[1] = 0;
      }
      if (digitalRead(TRACK2_BUTTON) == LOW) {
	track = 2;
	address[2] = 65536;
      }
      mode &= ~PASSTHROUGH;
      mode |= RECORD;
      digitalWrite(A4, HIGH);
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
  }

  if (!(mode & RECORD)) {
    if ((!(mode & PLAY0)) && (digitalRead(TRACK0_BUTTON) == LOW) && (trackRecorded[0])) {
      mode &= ~PASSTHROUGH;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	// enter LOOP mode
	mode |= LOOP0;
	delayCounter[0] = trackDelay[0]; // force immediate play.
      } else {
	address[0] = 0;
	mode |= PLAY0;
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
    if ((!(mode & PLAY1)) && (digitalRead(TRACK1_BUTTON) == LOW) && (trackRecorded[1])) {
      mode &= ~PASSTHROUGH;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	// enter LOOP mode
	mode |= LOOP1;
	delayCounter[1] = trackDelay[1]; // force immediate play
      } else {
	address[1] = 0;
	mode |= PLAY1;
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
    if ((!(mode & PLAY2)) && (digitalRead(TRACK2_BUTTON) == LOW) && (trackRecorded[2])) {
      mode &= ~PASSTHROUGH;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	// enter LOOP mode
	mode |= LOOP2;
	delayCounter[2] = trackDelay[2]; // force immediate play
      } else {
	address[2] = 65536;
	mode |= PLAY2;
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }

    // disable logic for loop mode
    if ((mode & LOOP0) && (digitalRead(LOOP_BUTTON) == HIGH) && (digitalRead(TRACK0_BUTTON) == LOW)) {
      mode &= ~LOOP0;
      mode &= ~PLAY0;
    }
    if ((mode & LOOP1) && (digitalRead(LOOP_BUTTON) == HIGH) && (digitalRead(TRACK1_BUTTON) == LOW)) {
      mode &= ~LOOP1;
      mode &= ~PLAY1;
    }
    if ((mode & LOOP2) && (digitalRead(LOOP_BUTTON) == HIGH) && (digitalRead(TRACK2_BUTTON) == LOW)) {
      mode &= ~LOOP2;
      mode &= ~PLAY2;
    }
  }


  if ((mode & RECORD) || (exhausted)) {
    if (((track == 0) && (digitalRead(TRACK0_BUTTON) == HIGH)) || 
	((track == 1) && (digitalRead(TRACK1_BUTTON) == HIGH)) || 
	((track == 2) && (digitalRead(TRACK2_BUTTON) == HIGH))) {
      // recording stopped
      recordEndTime = millis();
      if (recordEndTime - recordStartTime < 20) {
	// debounce
	return;
      }
      trackRecorded[track] = true;
      exhausted = false;
      endAddress[track] = address[track];
      if (batteryBackup) {
	// Write endAddress to EEPROM for battery backup use.
	byte a = track*3;
	EEPROM.write(a, (endAddress[track] >> 16) & 0xFF);
	EEPROM.write(a+1, (endAddress[track] >> 8) & 0xFF);
	EEPROM.write(a+2, endAddress[track] & 0xFF);
      }
#ifdef DEBUG
      Serial.print("track ");
      Serial.print(track);
      Serial.print(" recording time = ");
      Serial.print(recordEndTime - recordStartTime);
      Serial.print(", endAddress = ");
      Serial.println(endAddress[track]);
#endif
      mode &= ~RECORD;
      digitalWrite(A4, LOW);
      mode |= PASSTHROUGH;
      timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);
    }
  }

  


  if ((mode & PLAY0) && (!(mode & LOOP0)) && (digitalRead(TRACK0_BUTTON) == HIGH)) {
    // play button released
    mode &= ~PLAY0;
  }
  if ((mode & PLAY1) && (!(mode & LOOP1)) && (digitalRead(TRACK1_BUTTON) == HIGH)) {
    // play button released
    mode &= ~PLAY1;
  }
  if ((mode & PLAY2) && (!(mode & LOOP2)) && (digitalRead(TRACK2_BUTTON) == HIGH)) {
    // play button released
    mode &= ~PLAY2;
  }

}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;
  byte signal;
  byte sample;
  char track0 = 0;
  char track1 = 0;
  char track2 = 0;
  int mix;

  if (mode) {
    // if any mode bits on, then we have something to play 
    AudioHacker.writeDAC_8bit(playbackBuf);
  }


  if (mode & (PASSTHROUGH | RECORD)) {
    // Read ADC
    playbackBuf = AudioHacker.readADC_8bit();
  }

  if (mode & RECORD) {
    byte chip = (track == 0) ? 0 : 1;
    AudioHacker.writeSRAM(chip, address[track], playbackBuf);

    address[track]++;
    if (((track == 0) && (address[track] > 131071)) || 
	((track == 1) && (address[track] > 65535)) || 
	((track == 2) && (address[track] > 131071))) {
      // End of memory bank for this track. Stop recording.
      mode &= ~RECORD;
      exhausted = true;
    }
#ifdef DEBUG
    timer1End = TCNT1;
#endif
    return;
  }


  if ((mode & LOOP0) && (!(mode & PLAY0))) {
    delayCounter[0]++;
    if (delayCounter[0] >= trackDelay[0]) {
      // time to play this track
      address[0] = 0;
      mode |= PLAY0;
    }
  }
  if (mode & PLAY0) {
    sample = AudioHacker.readSRAM(0, address[0]);
    track0 = sample-128;

    address[0]++;
    if (address[0] > endAddress[0]) {
      delayCounter[0] = 0;
      address[0] = 0;
      if (mode & LOOP0) {
	mode &= ~PLAY0;
      }
    }
  }


  if ((mode & LOOP1) && (!(mode & PLAY1))) {
    delayCounter[1]++;
    if (delayCounter[1] >= trackDelay[1]) {
      // time to play this track
      address[1] = 0;
      mode |= PLAY1;
    }
  }
  if (mode & PLAY1) {
    sample = AudioHacker.readSRAM(1, address[1]);
    track1 = sample-128;

    address[1]++;
    if (address[1] > endAddress[1]) {
      delayCounter[1] = 0;
      address[1] = 0;
      if (mode & LOOP1) {
	mode &= ~PLAY1;
      }
    }
  }


  if ((mode & LOOP2) && (!(mode & PLAY2))) {
    delayCounter[2]++;
    if (delayCounter[2] >= trackDelay[2]) {
      // time to play this track
      address[2] = 65536;
      mode |= PLAY2;
    }
  }
  if (mode & PLAY2) {
    sample = AudioHacker.readSRAM(1, address[2]);
    track2 = sample-128;

    address[2]++;
    if (address[2] > endAddress[2]) {
      delayCounter[2] = 0;
      address[2] = 65536;
      if (mode & LOOP2) {
	mode &= ~PLAY2;
      }
    }
  }


  if (!(mode & (PASSTHROUGH | RECORD))) {
    // mix tracks
    mix = track0 + track1 + track2;
    // clip
    if (mix < -128) {
      mix = -128;
    } else {
      if (mix > 127) {
	mix = 127;
      }
    }
    mix += 128; // adjust back to range [0,255]
    playbackBuf = mix;
  }


#ifdef DEBUG
  timer1End = TCNT1;
#endif
}


