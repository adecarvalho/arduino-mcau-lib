/*
  Chiplib3
  A.DeCarvalho
 version 11 Octobre 2015
 - change DigitalOut
*/
#ifndef CHIP_LIB_3_H
#define CHIP_LIB_3_H

#include <Arduino.h>
#include <SPI.h>

//const and define
const uint8_t PULL_UP_ENABLE=1;
const uint8_t PULL_UP_DISABLE=0;

#define NOTE_R      0
// Notes Octave 3
#define NOTE_C3     131
#define NOTE_CS3    138
#define NOTE_D3     147
#define NOTE_DS3    155
#define NOTE_E3     165
#define NOTE_F3     175
#define NOTE_FS3    185
#define NOTE_G3     196
#define NOTE_GS3    208
#define NOTE_A3     220
#define NOTE_AS3    233
#define NOTE_B3     247

// Notes Octave 4
#define NOTE_C4     262
#define NOTE_CS4    277
#define NOTE_D4     294
#define NOTE_DS4    311
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_FS4    370
#define NOTE_G4     392
#define NOTE_GS4    415
#define NOTE_A4     440
#define NOTE_AS4    466
#define NOTE_B4     494

// Notes Octave 5
#define NOTE_C5     523
#define NOTE_CS5    554
#define NOTE_D5     587
#define NOTE_DS5    622
#define NOTE_E5     660
#define NOTE_F5     699
#define NOTE_FS5    740
#define NOTE_G5     784
#define NOTE_GS5    830
#define NOTE_A5     880
#define NOTE_AS5    933
#define NOTE_B5     988

// Notes Octave 6
#define NOTE_C6     1046
#define NOTE_CS6    1107
#define NOTE_D6     1173
#define NOTE_DS6    1245
#define NOTE_E6     1317
#define NOTE_F6     1397
#define NOTE_FS6    1480
#define NOTE_G6     1567
#define NOTE_GS6    1661
#define NOTE_A6     1760
#define NOTE_AS6    1865
#define NOTE_B6     1976

// Notes Octave 7
#define NOTE_C7     2096
#define NOTE_CS7    2217
#define NOTE_D7     2347
#define NOTE_DS7    2487
#define NOTE_E7     2638
#define NOTE_F7     2793
#define NOTE_FS7    2959
#define NOTE_G7     3135
#define NOTE_GS7    3322
#define NOTE_A7     3521
#define NOTE_AS7    3731
#define NOTE_B7     3953

///////////////////////
//AnalogIn
const uint8_t PS_128=_BV(ADPS2) | _BV(ADPS1) | (_BV(ADPS0));
const uint8_t PS_64=_BV(ADPS2)| _BV(ADPS1);
const uint8_t PS_32= _BV(ADPS2) | _BV(ADPS0);
const uint8_t PS_16=_BV(ADPS2);

//Note
const int tab_A[]= {
  NOTE_A3,NOTE_A4,NOTE_A5,NOTE_A6,NOTE_A7};
const int tab_AS[]= {
  NOTE_AS3,NOTE_AS4,NOTE_AS5,NOTE_AS6,NOTE_AS7};
const int tab_B[]= {
  NOTE_B3,NOTE_B4,NOTE_B5,NOTE_B6,NOTE_B7};
const int tab_BS[]= {
  NOTE_C3,NOTE_C4,NOTE_C5,NOTE_C6,NOTE_C7};
const int tab_C[]= {
  NOTE_C3,NOTE_C4,NOTE_C5,NOTE_C6,NOTE_C7};
const int tab_CS[]= {
  NOTE_CS3,NOTE_CS4,NOTE_CS5,NOTE_CS6,NOTE_CS7};
const int tab_D[]= {
  NOTE_D3,NOTE_D4,NOTE_D5,NOTE_D6,NOTE_D7};
const int tab_DS[]= {
  NOTE_DS3,NOTE_DS4,NOTE_DS5,NOTE_DS6,NOTE_DS7};
const int tab_E[]= {
  NOTE_E3,NOTE_E4,NOTE_E5,NOTE_E6,NOTE_E7};
const int tab_ES[]= {
  NOTE_F3,NOTE_F4,NOTE_F5,NOTE_F6,NOTE_F7};
const int tab_F[]= {
  NOTE_F3,NOTE_F4,NOTE_F5,NOTE_F6,NOTE_F7};
const int tab_FS[]= {
  NOTE_FS3,NOTE_FS4,NOTE_FS5,NOTE_FS6,NOTE_FS7};
const int tab_G[]= {
  NOTE_G3,NOTE_G4,NOTE_G5,NOTE_G6,NOTE_G7};
const int tab_GS[]= {
  NOTE_GS3,NOTE_GS4,NOTE_GS5,NOTE_GS6,NOTE_GS7};

#define isDigit(n)  (n>='0' && n<= '9')

//-----------
// DigitalOut
//----------------
class DigitalOut {
public:

  DigitalOut(uint8_t pin);
  void begin(void);
  virtual ~DigitalOut(void);
  void write(uint8_t value);
  uint8_t read(void) const;
  void operator=(uint8_t value);

protected:
  uint8_t myPin;
  uint8_t myBit;
  uint8_t myPort;
  volatile uint8_t *myOutput;
  uint8_t myValue;
};
//---------
// DigitalIn
//---------------
class DigitalIn {
public:
  DigitalIn(uint8_t pin);
  virtual ~DigitalIn(void);
  void begin(void);
  uint8_t read(void);

protected:

  uint8_t myPin;
  uint8_t myValue;

};
//------------------------
// AnalogIn vref=5V N=1023
//--------------
class AnalogIn {
public:

  AnalogIn(uint8_t pin);

  virtual ~AnalogIn(void);
  void begin(void);
  int16_t read(void);


protected:
  uint8_t myIn;
  int16_t myVal;
};
//----------------------
// AnalogOut via MCP4911
//---------------
class AnalogOut {

public:

  AnalogOut(uint8_t pinSS=53,uint8_t pinLdac=49,uint8_t clock_div=0x01);

  virtual ~AnalogOut(void);
  void write(int Nvalue);
  void begin(void);

protected:

  uint8_t myClockDiv;
  uint8_t mySSPin;
  uint8_t myLDACPin;

  int16_t myValue;
};
//---------
// ButtonIn
//--------------
class ButtonIn {

public:

  ButtonIn(uint8_t pin=9,uint8_t pull=PULL_UP_ENABLE);

  void begin(void);

  void waitForPress();
  void waitForRelease();
  void waitForPressAndRelease();
  boolean isPressed();
  virtual ~ButtonIn();

private:
  uint8_t myPin;
  uint8_t myPull;
  uint8_t myState;
};
//----------
// SoftTimer
//---------------
class SoftTimer {

public:
  SoftTimer(void);

  void update(void);
  void attach(void (*callback)(void),unsigned long periodMs);
  void start(void);
  void stop(void);
  virtual ~SoftTimer(void);

protected:

  unsigned long myLastTime;
  unsigned long myPeriod;
  bool isAction;
  void (*myCallback)(void);

};
//---------------
// Note for sound
//----------
class Note {

public:

  Note(char _note,bool _isSharp,char _octave,int _durationMs);

  Note(const char* _str,int _durationMs);

  virtual ~Note(void);

  int getNoteFreq(void) const;

  char getNoteChar(void) const;

  char getNoteOctave(void) const;

  int getNoteDuration(void) const;


protected:

  int calculateFreq(void);
  void parse(const char* _str);
  char myNoteChar;
  bool myIsSharp;
  char myOctave;  // 3 to 7
  int myDuration;
  int myFrequency;

};
//---------------
// device Buzzer
//-------------
class Buzzer {

public:

  Buzzer(char _pin=3);
  virtual ~Buzzer();
  void begin(void);

  void sound(const Note* _note);
  void music(const char* pmusic);
  boolean isMusicFinish(void) const;

protected:
  char myPin;
  boolean myIsMusicFinish;
};
//--------------------------------
#endif




