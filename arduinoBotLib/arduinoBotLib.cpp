#include "arduinoBotLib.h"

//************************
ArduinoBot::ArduinoBot(int *tabSensor, uint8_t pinLed, uint8_t pinBuzzer, uint8_t pinSensorSharp) : myTabSensor(tabSensor),
                                                                                                    myPinLed(pinLed), myPinBuzzer(pinBuzzer), myPinSensorSharp(pinSensorSharp)
{
  myMotorOffset = 0.0;
  myLeftSpeed = 0;
  myRightSpeed = 0;
  myNumericPosition = NO_LINE;
  myCapteurPosition = NO_LINE;
}
//**************************
void ArduinoBot::begin()
{
  uint8_t MuxPins[] = {MUXA, MUXB, MUXC};
  this->IRs.begin(MuxPins, MUX_IN, 3);
  pinMode(MUXI, INPUT);
  digitalWrite(MUXI, LOW);

  pinMode(IN_A1, OUTPUT);
  pinMode(IN_A2, OUTPUT);
  pinMode(IN_B1, OUTPUT);
  pinMode(IN_B2, OUTPUT);
  //
  pinMode(myPinLed, OUTPUT);
  digitalWrite(myPinLed, 0);
  //
  pinMode(myPinBuzzer, OUTPUT);
  digitalWrite(myPinBuzzer, 0);
}
//**************************
//***************************
// Led
//*********************************
void ArduinoBot::setLed(bool state)
{
  if (state)
  {
    digitalWrite(myPinLed, HIGH);
  }
  else
  {
    digitalWrite(myPinLed, LOW);
  }
}
//*************************
//
// Buzzer
//**************************
void ArduinoBot::buzzer(int frequency, int time_ms)
{
  if (frequency < 131)
    frequency = 131;
  if (frequency > 3953)
    frequency = 3953;
  //
  unsigned long peri = 1000000 / frequency;
  boolean state = false;

  for (unsigned long len = 0; len < (long)time_ms * 1000; len += peri)
  {
    state = !state;
    digitalWrite(myPinBuzzer, state);
    delayMicroseconds(peri - 50);
  }
  //
  delay(time_ms);

  digitalWrite(myPinBuzzer, LOW);
}
//*************************
//**************************
//
//motors
//*******************************************************
void ArduinoBot::setMotorSpeeds(int left, int right, int ms)
{
  myLeftSpeed = left;
  myRightSpeed = right;

  motorAdjustement();
  //
  if (myMotorOffset < 0)
    myRightSpeed *= (1 + myMotorOffset);
  else
    myLeftSpeed *= (1 - myMotorOffset);
  //
  ///
  if (myRightSpeed >= 0)
  {
    analogWrite(IN_A1, myRightSpeed);
    analogWrite(IN_A2, 0);
  }
  else
  {
    analogWrite(IN_A1, 0);
    analogWrite(IN_A2, -myRightSpeed);
  }
  ///
  if (myLeftSpeed >= 0)
  {
    analogWrite(IN_B1, myLeftSpeed);
    analogWrite(IN_B2, 0);
  }
  else
  {
    analogWrite(IN_B1, 0);
    analogWrite(IN_B2, -myLeftSpeed);
  }
  ///
  if (ms > 0)
  {
    delay(ms);
    setMotorStop(0);
  }
}
//********************************
void ArduinoBot::setMotorSpeeds(int left, int right)
{
  myLeftSpeed = left;
  myRightSpeed = right;

  motorAdjustement();
  //
  if (myMotorOffset < 0)
    myRightSpeed *= (1 + myMotorOffset);
  else
    myLeftSpeed *= (1 - myMotorOffset);
  //
  ///
  if (myRightSpeed >= 0)
  {
    analogWrite(IN_A1, myRightSpeed);
    analogWrite(IN_A2, 0);
  }
  else
  {
    analogWrite(IN_A1, 0);
    analogWrite(IN_A2, -myRightSpeed);
  }
  ///
  if (myLeftSpeed >= 0)
  {
    analogWrite(IN_B1, myLeftSpeed);
    analogWrite(IN_B2, 0);
  }
  else
  {
    analogWrite(IN_B1, 0);
    analogWrite(IN_B2, -myLeftSpeed);
  }
  ///
}
//********************************
void ArduinoBot::setMotorStop(int ms)
{
  myLeftSpeed = 0;
  myRightSpeed = 0;

  analogWrite(IN_A1, 255);
  analogWrite(IN_A2, 255);

  analogWrite(IN_B1, 255);
  analogWrite(IN_B2, 255);

  if (ms > 0)
    delay(ms);
}
//
//**********************************
int ArduinoBot::motorsReadLeftSpeed() const
{
  return this->myLeftSpeed;
}
//***********************************
int ArduinoBot::motorsReadRightSpeed() const
{
  return this->myRightSpeed;
}
//*********************************
//IR
//**********************************
void ArduinoBot::sensorRead()
{
  for (int i = 0; i < 5; i++)
  {
    this->myTabSensor[i] = sensorNumRead(i);
  }
}
//****************************************
int ArduinoBot::sensorNumRead(uint8_t num)
{
  IRs.selectPin(num);

  return IRs.getAnalogValue();
}
//************************************
void ArduinoBot::sensorPrintToSerial()
{
  for (int i = 0; i < 5; i++)
  {
    Serial.print(this->myTabSensor[i]);
    Serial.print(" : ");
  }
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("c0 : c1  : c2 : c3 : c4 ");
  //
  delay(500);
}
//*************************************
void ArduinoBot::motorAdjustement(void)
{
  myMotorOffset = map(analogRead(TRIM), 0, 1023, -30, 30) / 100.0;
}
//*************************************************
uint8_t ArduinoBot::codenameToAPin(uint8_t pin)
{
  switch (pin)
  {
  case TK1:
    return A0;
  //
  case TK2:
    return A1;
  //
  case TK3:
    return A6;
  //
  case TK4:
    return A11;
    //
  }
  return 0;
}
//***********************************************************
void ArduinoBot::digitalTkWrite(uint8_t pin, bool value)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value);
}
//*********************************************
bool ArduinoBot::digitalTkRead(uint8_t pin)
{
  pinMode(pin, INPUT);
  bool val = digitalRead(pin);
  return val;
}
//********************************************
int ArduinoBot::analogTkRead(uint8_t pin)
{
  pinMode(pin, INPUT);
  uint8_t broche = codenameToAPin(pin);
  int value = analogRead(broche);
  return value;
}
//**********************************************
int ArduinoBot::sharpReadDistance_mm()
{

  int val = analogTkRead(myPinSensorSharp);

  val = (5000.0 * val) / 1023.0;

  if (val < 200)
    return tabSharp[0];

  //
  if (val > 3000)
    return tabSharp[NB_ELEMENT - 1];

  int indice = (val / 200) - 1;

  return tabSharp[indice];
}
//**********************************************
