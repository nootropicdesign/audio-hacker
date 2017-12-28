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

#define MAX_UINT 65535
#define SRAM_SS 10
#define DAC_SS 9
#define ADC_SS 8
#define MAX_ADDR 131067 // last address for 3-byte storage of 2 samples

#define OFF 0
#define PASSTHROUGH 1
#define RECORD 2
#define PLAYBACK 3
#define RECORD_DONE 4
#define PLAY_BUTTON 6
#define RECORD_BUTTON 7


unsigned int playbackBuf = 2048;
volatile unsigned int tEven;
volatile unsigned int tOdd;
volatile int maxSignal = 0;
unsigned int sampleRate;
unsigned int startCounter;
volatile long address = 0;
volatile long endAddress = 0;;
volatile byte mode = PASSTHROUGH;
boolean mix = false;
int loopCounter = 0;
byte signalVolume;
byte sampleVolume;
int writeBuf;
int readBuf;
boolean evenCycle = true;
unsigned int recordStart;
void setup() {
  Serial.begin(115200);        // connect to the serial port

  Serial.println("setting pin modes...");
  pinMode(13, OUTPUT); // SCK
  pinMode(12, INPUT);  // MISO
  pinMode(11, OUTPUT); // MOSI

  pinMode(SRAM_SS, OUTPUT);
  pinMode(DAC_SS, OUTPUT);
  pinMode(ADC_SS, OUTPUT);

  pinMode(RECORD_BUTTON, INPUT);
  pinMode(PLAY_BUTTON, INPUT);
  digitalWrite(RECORD_BUTTON, HIGH);
  digitalWrite(PLAY_BUTTON, HIGH);


  Serial.println("deselecting devices...");
  digitalWrite(SRAM_SS, HIGH);
  digitalWrite(DAC_SS, HIGH);
  digitalWrite(ADC_SS, HIGH);

  Serial.println("configuring SPI...");
  // Configure SPI
  SPCR |= _BV(SPE);    // enable SPI
  SPCR &= ~_BV(SPIE);  // SPI interrupts off
  SPCR &= ~_BV(DORD);  // MSB first
  SPCR |= _BV(MSTR);   // SPI master mode
  SPCR &= ~_BV(CPOL);  // leading edge rising
  SPCR &= ~_BV(CPHA);  // sample on leading edge
  SPCR &= ~_BV(SPR1);  // speed = clock/4
  SPCR &= ~_BV(SPR0);  
  SPSR |= _BV(SPI2X);  // 2X speed



  Serial.println("configuring timer1...");
  // Disable Timer1 interrupt
  TIMSK1 &= ~_BV(TOIE1);

  TCCR1A = 0;
  TCCR1B = 0;

  // Set prescalar to 1;
  TCCR1B |= _BV(CS10);

  //TIMSK0 &= ~_BV(TOIE0); // disable Timer0

  sampleRate = 22050;
  startCounter = MAX_UINT - (16000000 / sampleRate);

  // Enable Timer1 interrupt
  TIMSK1 |= _BV(TOIE1);



  Serial.println("setup complete.");
}

void loop() {

  //sampleRate = map(analogRead(0), 0, 1023, 16000, 25000);
  // compute the start value for counter1 to achieve the chosen sample rate
  //startCounter = MAX_UINT - (16000000 / sampleRate);

  int a1 = analogRead(1);
  if (a1 < 512) {
    sampleVolume = map(a1, 0, 511, 0, 10);
    signalVolume = 10;
  } else {
    sampleVolume = 10;
    signalVolume = map(a1, 512, 1023, 10, 0);
  }

  if (loopCounter++ == 2000) {
    loopCounter = 0;

    /*
    Serial.print("mode = ");
    Serial.print(mode);
    Serial.print("    address = ");
    Serial.print(address);
    Serial.print("    endAddress = ");
    Serial.print(endAddress);
    Serial.println();
    */


    Serial.print("sampleRate = ");
    Serial.print(sampleRate);
    Serial.print(" Hz");

    Serial.print("   even cycles remaining = ");
    Serial.print(MAX_UINT - tEven);
    Serial.print("   odd cycles remaining = ");
    Serial.print(MAX_UINT - tOdd);


    Serial.print("   signalVolume = ");
    Serial.print(signalVolume);
    Serial.print("   sampleVolume = ");
    Serial.print(sampleVolume);

    Serial.println();

  }

  if ((mode == OFF) || (mode == PASSTHROUGH)) {
    if (digitalRead(RECORD_BUTTON) == LOW) {
      recordStart = millis();
      mode = RECORD;
      address = 0;
    }
    if (digitalRead(PLAY_BUTTON) == LOW) {
      mode = PLAYBACK;
      address = 0;
    }
  }

  if (mode == RECORD) {
    if (digitalRead(RECORD_BUTTON) == HIGH) {
      // record button released
      endAddress = address;
      mode = PASSTHROUGH;
      address = 0;
    }
  }

  if (mode == RECORD_DONE) {
    if (recordStart != 0) {
      Serial.print("recording time = ");
      Serial.println(millis() - recordStart);
      recordStart = 0;
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
    }
  }



}

inline byte spiWrite(byte data) {
  SPDR = data;
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

inline byte spiRead() {
  SPDR = 0xFF; // start SPI clock
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = startCounter;
  int signal;
  int sample;
  byte highByte;
  byte lowByte;

  if (mode != RECORD_DONE) {
    //  if ((mode != RECORD_DONE) && (mode != RECORD)) {
    // Write to DAC
    PORTB &= ~0x02;  // select
    byte dacConfig = 0x3; // DACa, unbuffered, gain=1, not shutdown
    spiWrite((dacConfig << 4) | ((playbackBuf >> 8) & 0xF)); // 4 config bits and bits 11-8
    spiWrite(playbackBuf); // bits 7-0
    PORTB |= 0x02;   // deselect
  }


  if (!(((mode == PLAYBACK) && (!mix)) || (mode == RECORD_DONE))) {
    // Don't read ADC if we are playing back without mixing or if we are not done recording
    // Read ADC
    SPCR |= _BV(SPR0);  // set SPI clock to clock/16 with SPI2X = 2MHz
    PORTB &= ~0x01;  // select
    spiWrite(0x1);   // start bit
    highByte = spiWrite(0xA0); // SGL,ODD,MSBF
    lowByte = spiRead();
    PORTB |= 0x01;   // deselect
    signal = highByte & 0x0F;
    signal = (signal << 8) | lowByte;
    SPCR &= ~_BV(SPR0);  // restore clock to clock/4 with SPI2X = 8MHz
  }

  if (mode == RECORD) {
    if (evenCycle) {
      // we only write to memory on odd cycles, so buffer the sampled signal.
      writeBuf = signal;
    } else {
      // Write to SRAM
      PORTB &= ~0x04;           // select
      spiWrite(0x02);           // write mode
      spiWrite(address >> 16);  // address
      spiWrite(address >> 8);
      spiWrite(address);
      writeBuf = writeBuf<<4;
      spiWrite(writeBuf >> 8);    // buffer MSB
      spiWrite(writeBuf | (signal >> 8)); // buffer least sig and signal most sig 4 bits
      spiWrite(signal);         // signal LSB
      PORTB |= 0x04;            // deselect

      address += 3;
      if (address > MAX_ADDR) {
	// end of memory, stop recording
	mode = RECORD_DONE;
	endAddress = address;
	address = 0; // loop around to beginning of memory
      }
    }
  }


  if (mode == PLAYBACK) {
    if (evenCycle) {
      // Read from SRAM
      PORTB &= ~0x04;  // select
      spiWrite(0x03);           // read mode
      spiWrite(address >> 16);  // address
      spiWrite(address >> 8);
      spiWrite(address);
      SPSR &= ~_BV(SPI2X); // 2X speed off for reading
      highByte = spiRead();
      lowByte = spiRead();
      sample = highByte;
      sample = (sample << 4) | (lowByte >> 4);
      highByte = lowByte;
      lowByte = spiRead();
      readBuf = highByte & 0x0F;
      readBuf = (readBuf << 8) | lowByte;
      PORTB |= 0x04;   // deselect
      SPSR |= _BV(SPI2X); // 2X speed on

      address += 3;
      if ((address == endAddress) || (address > MAX_ADDR)) {
	address = 0;
      }
    } else {
      sample = readBuf;
    }

    if (mix) {
      signal -= 2048;
      sample -= 2048;

      // Adjust signal volume
      if (signalVolume == 0) {
	// no sound
	signal = 0;
      } else {
	if (signalVolume <= 10) {
	  byte bitsToShift = ((10-signalVolume) >> 1);
	  int tmp = signal >> (bitsToShift+2);
	  signal = signal >> bitsToShift;
	  if (signalVolume & 0x1) {
	    signal = signal - tmp;
	  }
	}
      }

      // Adjust sample volume
      if (sampleVolume == 0) {
	// no sound
	sample = 0;
      } else {
	if (sampleVolume <= 10) {
	  byte bitsToShift = ((10-sampleVolume) >> 1);
	  int tmp = sample >> (bitsToShift+2);
	  sample = sample >> bitsToShift;
	  if (sampleVolume & 0x1) {
	    sample = sample - tmp;
	  }
	}
      }

      signal += sample;

      // clip
      if (signal < -2048) {
	signal = -2048;
      } else {
	if (signal > 2047) {
	  signal = 2047;
	}
      }
      signal += 2048;
    } else {
      signal = sample;
    }
  } // PLAYBACK



  playbackBuf = signal;

  if (evenCycle) {
    tEven = TCNT1;
  } else {
    tOdd = TCNT1;
  }
  evenCycle = !evenCycle;
}


