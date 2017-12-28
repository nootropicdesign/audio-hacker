/*
  Audio Hacker Library
  Copyright (C) 2015 nootropic design, LLC
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
  Example of digital filtering.
  Button on digital pin 4 cycles through 3 filter types: low pass, band pass,
  and high pass.
  Button on digital pin 5 enables/disables audio.
  Pot on A0 controls filter cutoff frequency.
  Pot on A1 controls filter resonance.
  Resonant filter adapted from http://www.musicdsp.org/archive.php?classid=3#29
*/

#include <AudioHacker.h>

//#define DEBUG
#define ON_OFF_BUTTON 5
#define FILTER_SELECT_BUTTON 4
#define LED1 A4
#define LED2 A5
#define LOW_PASS 0
#define BAND_PASS 1
#define HIGH_PASS 2
unsigned int playbackBuf = 2048;
unsigned int sampleRate = 22050;
unsigned int timer1Start;
volatile unsigned int timer1End;
boolean on = true;

int filterCutoff = 255;
int filterResonance = 255;
long feedback;
int buf0 = 0;
int buf1 = 0;
byte filterType = LOW_PASS;

unsigned long lastDebugPrint = 0;

void setup() {
#ifdef DEBUG
  Serial.begin(115200);        // connect to the serial port
#endif

  pinMode(ON_OFF_BUTTON, INPUT_PULLUP);
  pinMode(FILTER_SELECT_BUTTON, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, LOW);

  timer1Start = UINT16_MAX - (F_CPU / sampleRate);
  AudioHacker.begin();
}

void loop() {

  if (digitalRead(ON_OFF_BUTTON) == LOW) {
    on = !on;
    setLEDs();
    delay(20);
    while (digitalRead(ON_OFF_BUTTON) == LOW); // wait for release
    delay(20);
  }

  if ((digitalRead(FILTER_SELECT_BUTTON) == LOW) && (on)) {
    filterType++;
    if (filterType > HIGH_PASS) {
      filterType = LOW_PASS;
    }
    setLEDs();
    delay(20);
    while (digitalRead(FILTER_SELECT_BUTTON) == LOW); // wait for release
    delay(20);
  }

  filterCutoff = analogRead(0) >> 2;
  filterResonance = analogRead(1) >> 2;

  feedback = (long)filterResonance + (long)(((long)filterResonance * ((int)255 - (255-filterCutoff))) >> 8);

#ifdef DEBUG
  if ((millis() - lastDebugPrint) >= 500) {
    lastDebugPrint = millis();

    Serial.print("cycles remaining = ");
    Serial.print(UINT16_MAX - timer1End);
    Serial.println();
  }
#endif

  delay(50);
}

void setLEDs() {
  if (on) {
    if (filterType == LOW_PASS) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
    } 
    if (filterType == BAND_PASS) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
    }
    if (filterType == HIGH_PASS) {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
    }
  } else {
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
  }
    
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = timer1Start;

  if (on) {
    AudioHacker.writeDAC(playbackBuf);
    int reading = AudioHacker.readADC() - 2048;

    int highPass = reading - buf0;
    int bandPass = buf0 - buf1;
    int tmp = highPass + (feedback * bandPass >> 8);
    buf0 += ((long)filterCutoff * tmp) >> 8;
    buf1 += ((long)filterCutoff * (buf0 - buf1)) >> 8;
    switch (filterType) {
    case LOW_PASS:
      reading = buf1;
      break;
    case BAND_PASS:
      reading = bandPass;
      break;
    case HIGH_PASS:
      reading = highPass;
      break;
    }
	  

    playbackBuf = reading + 2048;
  }

#ifdef DEBUG
  timer1End = TCNT1;
#endif

}


