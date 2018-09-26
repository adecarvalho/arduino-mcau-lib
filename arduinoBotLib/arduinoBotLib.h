#ifndef ARDUINO_BOT_LIB_h
#define ARDUINO_BOT_LIB_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/**
ArduinoLib
version Avril 2018
*/
#include "Multiplexer.h"

//Sensor Sharp
#define NB_ELEMENT 15
const int tabSharp[NB_ELEMENT] = {320, 320, 220, 160, 130, 110, 90, 80, 70, 60, 55, 50, 45, 40, 30};

//sensorIR
#define SENSOR_NB 5

enum
{
  NO_LINE = -1,
  CROSS_T = -2,
  CROSS_RIGHT = -3,
  CROSS_LEFT = -4,
  ALL_BLACK = -5
};

//****************
class ArduinoBot
{
public:
  // constructor
  ArduinoBot(int *tabSensor, uint8_t pinLed, uint8_t pinBuzzer, uint8_t pinSensorSharp);

  void begin();

  //led
  void setLed(bool state);

  //buzzer
  void buzzer(int frequency, int time_ms);

  //motors
  void setMotorSpeeds(int left, int right, int ms);
  void setMotorSpeeds(int left, int right);
  //
  void setMotorStop(int ms);

  int motorsReadLeftSpeed() const;
  int motorsReadRightSpeed() const;

  //IR
  void sensorRead();
  void sensorPrintToSerial();
  int readNumericPosition() const;
  int readCapteurPosition() const;


  //Sharp
  int sharpReadDistance_mm();

  //TK
  void digitalTkWrite(uint8_t pin, bool value);
  bool digitalTkRead(uint8_t pin);
  int analogTkRead(uint8_t pin);

  //
private:
  Multiplexer IRs;
  float myMotorOffset;

  int sensorNumRead(uint8_t num);

  int *myTabSensor;
  int myNumericPosition;
  int myCapteurPosition;

  uint8_t myPinLed;
  uint8_t myPinBuzzer;
  uint8_t myPinSensorSharp;

  int myLeftSpeed;
  int myRightSpeed;

  void motorAdjustement(void);

  uint8_t codenameToAPin(uint8_t codename);
};

#endif
