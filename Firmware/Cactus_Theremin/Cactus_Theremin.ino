// Supproted chips: FDC2112, FDC2114, FDC2212, FDC2214
// Transmitts data via serial - use SerialPlot to draw graphs
//
// FDC2x1x hardware configuration:
// Component value as in default circuit form datasheet. (18uH inductor and 33pF cap)
//
//***** SD and ADDR pins tied to GND
// INTB pin not used
//
// ARDUINO <--> FDC
// A4 <-------> SDA
// A5 <-------> SCL

// TEENSY 3.2 <--> FDC
// 18 <-------> SDA
// 19 <-------> SCL

//
// !!!!!! Arduinos are mostly 5V. FDC chips are 3.3V, so either use 3.3V version of Arduino, like pro mini, or use level shifter on I2C bus.
//

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=188,240
AudioEffectEnvelope      envelope1;      //xy=371,237
AudioOutputI2S           i2s1;           //xy=565,241
AudioConnection          patchCord1(waveform1, envelope1);
AudioConnection          patchCord2(envelope1, 0, i2s1, 0);
AudioConnection          patchCord3(envelope1, 0, i2s1, 1);
AudioControlSGTL5000     audioShield;     //xy=586,175
// GUItool: end automatically generated code


// ### FDC
#include <Wire.h>
#include "FDC2214.h"
FDC2214 capsense(FDC2214_I2C_ADDR_0); // Use FDC2214_I2C_ADDR_1

#define CHAN_COUNT 1
#define DT 0 //Derivative threshold.
#define IT 1500 //Integration threshold.
#define UPPER_IT 2500 //Upper Integration threshold.
#define AVGN 3 //Degree of the IIR filter.
#define L .99 //Leakege Factor.
#define LED 13

// ###
void setup() {

    
  // Set up Audio
  AudioMemory(8);
  audioShield.enable();
  audioShield.volume(0.45);

  waveform1.pulseWidth(0.5);
  waveform1.begin(0.4, 220, WAVEFORM_SINE);

  envelope1.attack(250);
  envelope1.decay(50);
  envelope1.release(350);

  //setup LED and FDC
  pinMode(LED, OUTPUT);
  // ### Start I2C
  Wire.begin();
  //  Wire.setClock(400000L);

  // ### Start serial
  Serial.begin(115200);
  Serial.println("\nFDC2x1x test");

  // ### Start FDC
  // Start FDC2212 with 2 channels init
  bool capOk = capsense.begin(0x1, 0x4, 0x5, false); //setup first two channels, autoscan with 2 channels, deglitch at 10MHz, external oscillator

  if (capOk) Serial.println("Sensor OK");
  else Serial.println("Sensor Fail");

}



//IIR Filter
long average( unsigned long meassure, long avg_1, int n) {
  long avg = ((avg_1 * n) - avg_1 + meassure) / n;
  return avg;
}

bool playing;

void LedOn() {
  digitalWrite(LED, HIGH); //Target approaching!
  if (!playing)
    envelope1.noteOn();
  playing = true;
}

void LedOff() {
  digitalWrite(LED, LOW);

  if (playing)
    envelope1.noteOff();

  playing = false;
}

// ###
void loop() {
  long current = capsense.getReading28(0);
  long delta = 0, Integral = 0, Integral_1 = 0;
  long avg = 0, avg_1 = 0, last = 0;
  //avg = current;
  
  //wait for usb cap integral to drop 
  Serial.println("Initializing");
  delay(3000);
  
  for (;;) {

    last = current;// Shift register. The variable last will storage the current value for the next iteration.
    current = average(capsense.getReading28(0), last, AVGN);//New value for current return from IIR filter --> function average(meassure, avg_1, n)

    //Derivative integration algorithm
    delta = current - last;

    if (abs(delta) > DT) {
      Integral = Integral_1 + delta;
    }
    else {
      Integral = Integral_1;
    }

    if (abs(Integral) >= IT) {
      float mappedFrequency = map(abs(Integral),IT,IT*16,20,2000);
      
      waveform1.frequency(constrain(mappedFrequency,20,2000));
      LedOn();
      Integral_1 = Integral;
    }
    else {
      LedOff();
      Integral_1 = Integral * L;
    }

    Serial.println(abs(Integral));
  }
  // No point in sleeping
  //delay(100);
}
