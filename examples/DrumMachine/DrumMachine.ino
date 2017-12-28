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
  3-Track Drum Machine

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
*/

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

volatile byte mode = PASSTHROUGH;
unsigned long lastDebugPrint = 0;
boolean recordPressed;
unsigned int recordStartTime;
unsigned int recordEndTime;
boolean sampleRecorded = false;
volatile boolean trackRecorded[3];
byte track;
volatile unsigned int delayCounter[3];
unsigned int trackDelay[3];
volatile byte beatIndex[3];
byte beatCount[3]; // number of beats for each track
unsigned int beatDelay[3][4]; // [track][beat].  delay for track while playing this beat
boolean loopSetup[3];
byte volumeShift[3];
unsigned long lastBeatStart;
unsigned long beatDuration;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  trackRecorded[0] = false;
  trackRecorded[1] = false;
  trackRecorded[2] = false;
  beatCount[0] = 0;
  beatCount[1] = 0;
  beatCount[2] = 0;
  beatIndex[0] = 0;
  beatIndex[1] = 0;
  beatIndex[2] = 0;
  loopSetup[0] = false;
  loopSetup[1] = false;
  loopSetup[2] = false;

  recordingSampleRate = 16000; // todo: tune this or go down to 2 tracks.
  passthroughSampleRate = 22050;
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
  Serial.println("setup complete.");
#endif
}

// Design:
// trackDelay is not the delay between end of last play and beginning of next play.  It's delay between beginning of each beats.
// trackDelay is a cycle count.  So when I measure the time between beats, convert to cycles.
// press loop button then track button starts setting the beats.
// we enter loop mode when loop button is released after setting beats.


void loop() {

  volumeShift[0] = map(analogRead(0), 0, 1023, 8, 0);
  volumeShift[1] = map(analogRead(1), 0, 1023, 8, 0);
  volumeShift[2] = map(analogRead(2), 0, 1023, 8, 0);

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
    /*
    if (beatCount[0] > 0) {
      Serial.print("track 0 beats = ");
      Serial.println(beatCount[0]);
      //for(byte l=0;l<beatCount[0];l++) {
      for(byte l=0;l<4;l++) {
	Serial.println(beatDelay[0][l]);
      }
      Serial.println();
    }

    if (beatCount[1] > 0) {
      Serial.print("track 1 beats = ");
      Serial.println(beatCount[1]);
      for(byte l=0;l<beatCount[1];l++) {
	Serial.println(beatDelay[1][l]);
      }
      Serial.println();
    }

    if (beatCount[2] > 0) {
      Serial.print("track 2 beats = ");
      Serial.println(beatCount[2]);
      for(byte l=0;l<beatCount[2];l++) {
	Serial.println(beatDelay[2][l]);
      }
      Serial.println();
    }
    */

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
    // if PLAY0 is already set and LOOP0 is not, then this is not a new press of track button, because a release of the track button would have unset PLAY0.  wait, what if we are looping and it is setting PLAY0?  then LOOP0 is also set
    // we want this logic if:
    // any time except if we are playing and not looping.  That is, we are holding the track button down to play.
    // if we are looping and PLAY0 is set, it's ok to enter and start a new play of the track, even defining a new loop
    // enter if playing ONLY if looping.  Otherwise, it's a new press only if PLAY0 is unset.

    if ((digitalRead(TRACK0_BUTTON) == LOW) && (trackRecorded[0]) && (!((mode & PLAY0) && (!(mode & LOOP0))))) {
      // new press of track button
      mode &= ~PASSTHROUGH;
      address[0] = 0;
      mode |= PLAY0;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	mode &= ~LOOP0;
	if (!loopSetup[0]) {
	  // first beat in loop
	  lastBeatStart = millis();
	  loopSetup[0] = true;
	  beatCount[0] = 1;
	} else {
	  // not first beat in loop
	  beatDuration = millis() - lastBeatStart;
	  lastBeatStart = millis();
	  beatDelay[0][beatCount[0]-1] = beatDuration * (recordingSampleRate / 1000);
	  beatCount[0]++;
	}
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
    if ((loopSetup[0]) && (digitalRead(LOOP_BUTTON) == HIGH)) {
      // loop button released.  Enter loop mode for track0.
      loopSetup[0] = false;
      beatIndex[0] = beatCount[0];
      beatDuration = millis() - lastBeatStart;
      beatDelay[0][beatCount[0]-1] = beatDuration * (recordingSampleRate / 1000);
      trackDelay[0] = beatDelay[0][beatCount[0]-1];
      delayCounter[0] = trackDelay[0]; // force immediate play starting with beat 0
      mode |= LOOP0;
    }

    if ((digitalRead(TRACK1_BUTTON) == LOW) && (trackRecorded[1]) && (!((mode & PLAY1) && (!(mode & LOOP1))))) {
      mode &= ~PASSTHROUGH;
      address[1] = 0;
      mode |= PLAY1;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	if (!loopSetup[1]) {
	  // first beat in loop
	  lastBeatStart = millis();
	  loopSetup[1] = true;
	  beatCount[1] = 1;
	} else {
	  // not first beat in loop
	  unsigned long beatDuration = millis() - lastBeatStart;
	  lastBeatStart = millis();
	  beatDelay[1][beatCount[1]-1] = beatDuration * (recordingSampleRate / 1000);
	  beatCount[1]++;
	}
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
    if ((loopSetup[1]) && (digitalRead(LOOP_BUTTON) == HIGH)) {
      // loop button released.  Enter loop mode for track1.
      loopSetup[1] = false;
      beatIndex[1] = beatCount[1];
      beatDuration = millis() - lastBeatStart;
      beatDelay[1][beatCount[1]-1] = beatDuration * (recordingSampleRate / 1000);
      trackDelay[1] = beatDelay[1][beatCount[1]-1];
      delayCounter[1] = trackDelay[1]; // force immediate play starting with beat 0
      mode |= LOOP1;
    }


    if ((digitalRead(TRACK2_BUTTON) == LOW) && (trackRecorded[2]) && (!((mode & PLAY2) && (!(mode & LOOP2))))) {
      mode &= ~PASSTHROUGH;
      address[2] = 65536;
      mode |= PLAY2;
      if (digitalRead(LOOP_BUTTON) == LOW) {
	if (!loopSetup[2]) {
	  // first beat in loop
	  lastBeatStart = millis();
	  loopSetup[2] = true;
	  beatCount[2] = 1;
	} else {
	  // not first beat in loop
	  unsigned long beatDuration = millis() - lastBeatStart;
	  lastBeatStart = millis();
	  beatDelay[2][beatCount[2]-1] = beatDuration * (recordingSampleRate / 1000);
	  beatCount[2]++;
	}
      }
      timer1Start = UINT16_MAX - (F_CPU / recordingSampleRate);
    }
    if ((loopSetup[2]) && (digitalRead(LOOP_BUTTON) == HIGH)) {
      // loop button released.  Enter loop mode for track2.
      loopSetup[2] = false;
      beatIndex[2] = beatCount[2];
      beatDuration = millis() - lastBeatStart;
      beatDelay[2][beatCount[2]-1] = beatDuration * (recordingSampleRate / 1000);
      trackDelay[2] = beatDelay[2][beatCount[2]-1];
      delayCounter[2] = trackDelay[2]; // force immediate play starting with beat 0
      mode |= LOOP2;
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

  } // if not RECORD mode


  if (mode & RECORD) {
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
      endAddress[track] = address[track];
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

  if ((digitalRead(RECORD_BUTTON) == HIGH) && ((digitalRead(TRACK0_BUTTON) == LOW) || (digitalRead(TRACK1_BUTTON) == LOW) || (digitalRead(TRACK2_BUTTON) == LOW))) {
    digitalWrite(A5, HIGH);
  } else {
    digitalWrite(A5, LOW);
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
      endAddress[track] = address[track];
      trackRecorded[track] = true;
    }
#ifdef DEBUG
    timer1End = TCNT1;
#endif
    return;
  }


  if (mode & LOOP0) {
    delayCounter[0]++;
    if (delayCounter[0] >= trackDelay[0]) {
      // time to start the track over.
      delayCounter[0] = 0;
      address[0] = 0;
      mode |= PLAY0;  // fall through

      beatIndex[0]++;
      if (beatIndex[0] >= beatCount[0]) {
	beatIndex[0] = 0;
      }
      trackDelay[0] = beatDelay[0][beatIndex[0]];
    }
  }
  if (mode & PLAY0) {
    sample = AudioHacker.readSRAM(0, address[0]);
    track0 = sample-128;

    address[0]++;
    if (address[0] > endAddress[0]) {
      address[0] = 0;
      if (mode & LOOP0) {
	mode &= ~PLAY0;
      }
    }
  }


  if (mode & LOOP1) {
    delayCounter[1]++;
    if (delayCounter[1] >= trackDelay[1]) {
      // time to start the track over.
      delayCounter[1] = 0;
      address[1] = 0;
      mode |= PLAY1;

      beatIndex[1]++;
      if (beatIndex[1] >= beatCount[1]) {
	beatIndex[1] = 0;
      }
      trackDelay[1] = beatDelay[1][beatIndex[1]];
    }
  }
  if (mode & PLAY1) {
    sample = AudioHacker.readSRAM(1, address[1]);
    track1 = sample-128;

    address[1]++;
    if (address[1] > endAddress[1]) {
      address[1] = 0;
      if (mode & LOOP1) {
	mode &= ~PLAY1;
      }
    }
  }


  if (mode & LOOP2) {
    delayCounter[2]++;
    if (delayCounter[2] >= trackDelay[2]) {
      // time to play this track
      delayCounter[2] = 0;
      address[2] = 65536;
      mode |= PLAY2;

      beatIndex[2]++;
      if (beatIndex[2] >= beatCount[2]) {
	beatIndex[2] = 0;
      }
      trackDelay[2] = beatDelay[2][beatIndex[2]];
    }
  }
  if (mode & PLAY2) {
    sample = AudioHacker.readSRAM(1, address[2]);
    track2 = sample-128;

    address[2]++;
    if (address[2] > endAddress[2]) {
      address[2] = 65536;
      if (mode & LOOP2) {
	mode &= ~PLAY2;
      }
    }
  }

  track0 = track0 >> volumeShift[0];
  track1 = track1 >> volumeShift[1];
  track2 = track2 >> volumeShift[2];

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


