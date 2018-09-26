#ifndef ZUMO_BOT_LIB_H
#define ZUMO_BOT_LIB_H

#include <Arduino.h>

const byte buzzerpin = 3;
const byte ledpin = 13;
const byte buttonpin = 12;

const byte leftpwmpin=10;
const byte rightpwmpin=9;

const byte leftdirpin=8;
const byte rightdirpin=7;

const byte sensorcompin=A4;

class ZumoBot
{

  public:
    static ZumoBot *getInstance()
    {
        if (myinstance == 0)
        {
            myinstance = new ZumoBot();
        }
        return myinstance;
    }
    //
    void begin(int *tabSensorValues);
    //
    float getBatteryVoltage();
    //
    void setLed(bool state);
    //
    bool isButtonPressed();
    void waitButtonPressed();
    void waitButtonReleased();
    void waitButtonPressAndReleased();
    //
    void buzzer(int frequency, int time_ms);
    //
    void setMotorLeftSpeed(int speed);
    void setMotorRightSpeed(int speed);
    void setMotorStop(int ms);
    void setMotorSpeeds(int left, int right, int time_ms);
    //
    void sensorRead();
    void sensorActivate(bool state);
    void sensorPrintToSerial();

  private:
    ZumoBot();
    ~ZumoBot();
    //
    static ZumoBot *myinstance;
    //
    void beginLed();
    void beginButton();
    void beginBuzzer();
    void beginMotors();
    void beginSensor(int *tabValues);
    //
    int *myTabSensorValues;
    uint8_t *myTabSensorPins;
};

#endif