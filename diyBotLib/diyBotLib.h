#ifndef DIYBOT_LIB_H
#define DIYBOT_LIB_H

#include <Arduino.h>

#define TE_TICK 500 //Te=20ms pour servo
const byte myledpin = 2;
const byte mybuttonpin = 9;

const byte myleftpwm = 11;
const byte myrightpwm = 3;
const byte myleftdir = 13;
const byte myrightdir = 12;

const byte mytrigpin = A1;
const byte myservopin = 8;
const byte myechopin = A0;

class DiyBot
{
  public:
    //
    DiyBot();
    ~DiyBot();
    //
    void begin();
    //
    void setLed(bool state);
    //
    bool isButtonPressed();
    void waitButtonPressed();
    void waitButtonReleased();
    void waitButtonPressAndReleased();
    //
    void setMotorSpeeds(int left, int right, int ms);
    void setMotorSpeeds(int left, int right);
    void setMotorLeftSpeed(int speed);
    void setMotorRightSpeed(int speed);
    void setMotorStop(int ms);
    //
    void turretAction();
    int turretReadDistance_mm();
    void turretWriteAngle(int deg, int ms);

  private:
    //
    void beginLed();
    void beginButton();
    void beginMotor();
    void beginTurret();
    //
    bool myTrigOn;
    bool myServoOn;
    int myServoTick;
    int myAngle;
    //
    int myTabValDistance[4];
    uint8_t myPointeur;
};

#endif