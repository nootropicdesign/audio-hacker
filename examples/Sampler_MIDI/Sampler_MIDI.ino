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

  Requires the Arduino MIDI library:
  http://playground.arduino.cc/Main/MIDILibrary

  This sketch was used with the Rugged Circuits Flexible MIDI Shield
  with the enable pin set to D17 (A3).
  Other MIDI shields are NOT likely to work as they will have pin conflicts.
  Only the Rugged Circuits Flexible MIDI Shield is recommended.

  When a "noteOn" MIDI message is received, the sample is played at an 
  adjusted playback rate so that the frequency shifts to the requested note.

  See Audio Hacker project page for details.
  http://nootropicdesign.com/audiohacker/projects.html
 */

#include <AudioHacker.h>
#include <MIDI.h>

#define OFF 0
#define PASSTHROUGH 1
#define RECORD 2
#define PLAYBACK 3
#define RECORD_DONE 4
#define RECORD_BUTTON 5
#define PLAY_BUTTON 6
#define MIDI_ENABLE 17  // Used with Rugged Circuits MIDI Shield with enable pin set to D17
#define BASE_NOTE 60    // Middle C (C4)

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

byte lastNote = 0xFF;
float midi_freq[120];

void setup() {
  recordingSampleRate = DEFAULT_RECORDING_SAMPLE_RATE;
  passthroughSampleRate = DEFAULT_SAMPLE_RATE;
  timer1Start = UINT16_MAX - (F_CPU / passthroughSampleRate);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(PLAY_BUTTON, INPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(PLAY_BUTTON, HIGH);

  AudioHacker.begin();

  // Enable MIDI shield.
  pinMode(MIDI_ENABLE, OUTPUT);
  digitalWrite(MIDI_ENABLE, HIGH);

  // Initialize MIDI library.
  MIDI.begin(MIDI_CHANNEL_OMNI);    
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  // precompute frequencies
  for(int i=0;i<120;i++) {
    midi_freq[i] = 440.0 * (pow(2, (i-69)/12.0));
  }
}

void loop() {
  MIDI.read();

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
      playbackSampleRate = recordingSampleRate;
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
      sampleRecorded = true;
      recordStartTime = 0;
    }
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      mode = PASSTHROUGH;
    }
  }

  if (mode == PLAYBACK) {
    if ((digitalRead(PLAY_BUTTON) == HIGH) && (lastNote == 0xFF)) {
      // play button released or MIDI off.
      mode = PASSTHROUGH;
      address = 0;
      addressChipNumber = 0;
    } else {
      // compute the start value for counter1 to achieve the chosen playback rate
      timer1Start = UINT16_MAX - (F_CPU / playbackSampleRate);
    }
  }
}

void handleNoteOn(byte channel, byte note, byte velocity) { 
  if (velocity == 0) {
    // This acts like a NoteOff.
    if (note == lastNote) {
      mode = PASSTHROUGH;
      lastNote = 0xFF;
    }
  } else {
    // Alter the playback speed depending on the note played.  First compute a ratio
    // that describes the note frequency relative to the base note of middle C (C4).
    float ratio = 1.0 + ((midi_freq[note] - midi_freq[BASE_NOTE]) / midi_freq[BASE_NOTE]);
    // Set the playback rate to the rate it was recorded at multiplied by the ratio to speed
    // it up or slow it down.
    if (note == 72) ratio = 2.0;
    playbackSampleRate = (unsigned int)(recordingSampleRate * ratio);
    address = 0;
    addressChipNumber = 0;
    mode = PLAYBACK;

    /*
    long length = endAddress + (MAX_ADDR * endAddressChipNumber);
    byte n = 13;
    address = (note-60) * (length/n); // divide sample into n slices
    if (address > MAX_ADDR) {
      addressChipNumber = 1;
      address -= MAX_ADDR;
    } else {
      addressChipNumber = 0;
    }

    // round down to multiple of 3.  Two 1.5 byte samples are stored in each group of 3 bytes.
    address -= (address % 3);
    */

    lastNote = note;
  }
}

void handleNoteOff(byte channel, byte note, byte velocity) { 
  mode = PASSTHROUGH;
  lastNote = 0xFF;
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

  evenCycle = !evenCycle;
}


