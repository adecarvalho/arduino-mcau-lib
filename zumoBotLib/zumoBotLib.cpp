#include "zumoBotLib.h"

ZumoBot *ZumoBot::myinstance = 0;

uint8_t theSensorPins[6] = {4, A3, 11, A0, A2, 5};
//
ZumoBot::ZumoBot()
{
}
//
ZumoBot::~ZumoBot()
{
}
//
void ZumoBot::begin(int *tabSensorValues)
{
    this->beginSensor(tabSensorValues);
    //
    this->beginLed();
    //
    this->beginButton();
    //
    this->beginBuzzer();
    //
    this->beginMotors();
}
//
float ZumoBot:: getBatteryVoltage()
{
    float res=0;

    int num=analogRead(A1);
    res=(5.0*num)/1023.0;
    res=res*3.0/2.0;

    return res;
}
//
void ZumoBot::beginLed()
{
    pinMode(ledpin, OUTPUT); //led pin
    digitalWrite(ledpin, 0);
}
//
void ZumoBot::setLed(bool state)
{
    if (state)
    {
        digitalWrite(ledpin, 1);
    }
    else
    {
        digitalWrite(ledpin, 0);
    }
}
//
void ZumoBot::beginButton()
{
    pinMode(buttonpin, INPUT_PULLUP); //button pin
}
//
bool ZumoBot::isButtonPressed()
{
    bool res = false;

    if (digitalRead(buttonpin) == LOW)
    {
        res = true;
    }
    else
    {
        res = false;
    }

    return res;
}
//
void ZumoBot::waitButtonPressed()
{
    do
    {
        while (!this->isButtonPressed())
        {
        };
        delay(20);

    } while (!this->isButtonPressed());
}
//
void ZumoBot::waitButtonReleased()
{
    do
    {
        while (this->isButtonPressed())
        {
        };
        delay(20);

    } while (this->isButtonPressed());
}
//
void ZumoBot::waitButtonPressAndReleased()
{
    this->waitButtonPressed();
    this->waitButtonReleased();
}
//
void ZumoBot::beginBuzzer()
{
    pinMode(buzzerpin, OUTPUT); //buzzer pin
    digitalWrite(buzzerpin, 0);
}
//
void ZumoBot::buzzer(int frequency, int time_ms)
{
    if (frequency < 131)
        frequency = 131;
    //
    if (frequency > 3953)
        frequency = 3953;
    //
    unsigned long peri=1000000/frequency;
    boolean state=false;

    for(unsigned long len=0;len<(long)time_ms*1000;len+=peri)
    {
        state=!state;
        digitalWrite(buzzerpin,state);
        delayMicroseconds(peri-50);
    }

    //tone(3, frequency, time_ms);
    delay(time_ms);

    digitalWrite(buzzerpin, 0);
}
//
void ZumoBot::beginMotors()
{
    pinMode(leftpwmpin, OUTPUT); //left motor pwm pin
    pinMode(rightpwmpin, OUTPUT);  //right motor pwm pin
    pinMode(leftdirpin, OUTPUT);  //left dir pin
    pinMode(rightdirpin, OUTPUT);  //right dir pin
    //
    TCCR1A = 0b10100000;
    TCCR1B = 0b00010001;
    ICR1 = 400;
    OCR1A = 0;
    OCR1B = 0;
}
//
void ZumoBot::setMotorLeftSpeed(int speed)
{
    bool reverse = false;
    //
    if (speed < 0)
    {
        speed = -speed;
        reverse = true;
    }
    //
    if (speed > 400)
    {
        speed = 400;
    }
    //
    OCR1B = speed;

    if (reverse)
        digitalWrite(leftdirpin, 1);
    else
        digitalWrite(leftdirpin, 0);
}
//
void ZumoBot::setMotorRightSpeed(int speed)
{
    bool reverse = false;
    //
    if (speed < 0)
    {
        speed = -speed;
        reverse = true;
    }
    //
    if (speed > 400)
    {
        speed = 400;
    }
    //
    OCR1A = speed;

    if (reverse)
        digitalWrite(rightdirpin, 1);
    else
        digitalWrite(rightdirpin, 0);
}
//
void ZumoBot::setMotorStop(int ms)
{
    this->setMotorLeftSpeed(0);
    this->setMotorRightSpeed(0);
    if(ms >0)
    {
        delay(ms);
    }
}
//
void ZumoBot::setMotorSpeeds(int left, int right, int time_ms)
{
    this->setMotorLeftSpeed(left);
    this->setMotorRightSpeed(right);
    //
    if (time_ms > 0)
    {
        delay(time_ms);
        this->setMotorStop(0);
    }
}
//
void ZumoBot::beginSensor(int *tabValues)
{
    this->myTabSensorValues = tabValues;
    this->myTabSensorPins = theSensorPins;
    //
    pinMode(sensorcompin, OUTPUT); //sensor com
    digitalWrite(sensorcompin, LOW);

    delayMicroseconds(200);
}
//
void ZumoBot::sensorRead()
{
    //reset
    for (int i = 0; i < 6; i++)
    {
        this->myTabSensorValues[i] = 2000; //val max 2ms
        digitalWrite(this->myTabSensorPins[i], HIGH);
        pinMode(this->myTabSensorPins[i], OUTPUT);
    }
    //
    delayMicroseconds(10);
    //mise etat bas
    for (int i = 0; i < 6; i++)
    {
        pinMode(this->myTabSensorPins[i], INPUT);
        digitalWrite(this->myTabSensorPins[i], LOW);
    }
    //
    unsigned long startTime = micros();
    int mtime = 0;

    while (micros() - startTime < 2000)
    {
        mtime = micros() - startTime;
        for (int i = 0; i < 6; i++)
        {
            if (digitalRead(this->myTabSensorPins[i]) == LOW && mtime < this->myTabSensorValues[i])
            {
                this->myTabSensorValues[i] = mtime;
            }
        }
    }
}
//
void ZumoBot::sensorActivate(bool state)
{
    if (state)
    {
        digitalWrite(sensorcompin, HIGH);
    }
    else
    {
        digitalWrite(sensorcompin, LOW);
    }
    delayMicroseconds(200);
}
//
void ZumoBot::sensorPrintToSerial()
{
    Serial.println("----------------");
    for(int i=0;i<6;i++)
    {
        Serial.print("Sensor: ");
        Serial.print(i);
        Serial.print(">");
        Serial.println(this->myTabSensorValues[i]);
        delay(50);
    }
    delay(500);
}
//
