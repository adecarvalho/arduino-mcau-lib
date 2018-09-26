
#include "diyBotLib.h"

//
DiyBot::DiyBot()
{
}
//
DiyBot::~DiyBot()
{
}
//
void DiyBot::begin()
{
    this->beginLed();
    this->beginButton();
    this->beginMotor();
    this->beginTurret();
}
//
void DiyBot::setLed(bool state)
{
    if (state)
    {
        digitalWrite(myledpin, HIGH);
    }
    else
    {
        digitalWrite(myledpin, LOW);
    }
}
//
void DiyBot::beginLed()
{
    pinMode(myledpin, OUTPUT);
    digitalWrite(myledpin, LOW);
}
//
void DiyBot::beginButton()
{
    pinMode(mybuttonpin, INPUT_PULLUP); //button pin
}
//
bool DiyBot::isButtonPressed()
{
    bool res = false;

    if (digitalRead(mybuttonpin) == LOW)
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
void DiyBot::waitButtonPressed()
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
void DiyBot::waitButtonReleased()
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
void DiyBot::waitButtonPressAndReleased()
{
    this->waitButtonPressed();
    this->waitButtonReleased();
}
//
void DiyBot::beginMotor()
{
    pinMode(myleftdir,OUTPUT);
    pinMode(myrightdir,OUTPUT);
    pinMode(myleftpwm,OUTPUT);
    pinMode(myrightpwm,OUTPUT);

    analogWrite(myleftpwm,0);
    analogWrite(myrightpwm,0);
    digitalWrite(myleftdir,HIGH);
    digitalWrite(myrightdir,HIGH);
}
//
void DiyBot::setMotorLeftSpeed(int speed)
{
    bool reverse = false;
    //
    if (speed < 0)
    {
        reverse = true;
        speed = -speed;
    }
    //
    if (speed > 127)
    {
        speed = 127;
    }
    //
    if (reverse)
        digitalWrite(myleftdir, HIGH);
    else
        digitalWrite(myleftdir, LOW);
    //
    analogWrite(myleftpwm, speed);
}
//
void DiyBot ::setMotorRightSpeed(int speed)
{
    bool reverse = false;
    //
    if (speed < 0)
    {
        reverse = true;
        speed = -speed;
    }
    //
    if (speed > 127)
    {
        speed = 127;
    }
    //
    if (reverse)
        digitalWrite(myrightdir, HIGH);
    else
        digitalWrite(myrightdir, LOW);
    //
    analogWrite(myrightpwm, speed);
}
//
void DiyBot::setMotorSpeeds(int left, int right, int ms)
{
    this->setMotorSpeeds(left, right);

    delay(ms);

    if (ms > 0)
        this->setMotorStop(ms);
}
//
void DiyBot ::setMotorSpeeds(int left, int right)
{
    this->setMotorLeftSpeed(left);
    this->setMotorRightSpeed(right);
}
//
void DiyBot::setMotorStop(int ms)
{
    digitalWrite(myrightdir, LOW);
    digitalWrite(myleftdir, LOW);

    analogWrite(myleftpwm, 0);
    analogWrite(myrightpwm, 0);

    if (ms > 0)
    {
        delay(ms);
    }
}
//
void DiyBot::beginTurret()
{
    pinMode(mytrigpin, OUTPUT);

    pinMode(myservopin, OUTPUT);

    pinMode(myechopin, INPUT);
    //
    myTrigOn = false;
    myServoOn = false;
    myServoTick = 35; //90 deg
    myAngle = 90;
    myPointeur = 0;

    for (int i = 0; i < 4; i++)
    {
        myTabValDistance[i] = 0;
    }
    //inte TIMER
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    OCR1A = 640; //isr comp 40us
    TCCR1B |= _BV(WGM12);
    TCCR1B |= _BV(CS10);
    TIMSK1 |= _BV(OCIE1A);
    sei();
    //
}
//
void DiyBot::turretAction()
{
    static int nbTick = 0;
    static int nbEcho = 0;

    if (nbTick >= TE_TICK)
    {
        myTrigOn = false;
        myServoOn = false;
        nbTick = 0;

        myTabValDistance[myPointeur] = nbEcho;
        myPointeur++;
        if (myPointeur > 4)
        {
            myPointeur = 0;
        }
        return;
    }
    nbTick++;
    //////////////////////
    if (myTrigOn == false)
    {
        digitalWrite(mytrigpin, HIGH);
        myTrigOn = true;
        nbEcho = 0;
    }
    else
    {
        digitalWrite(mytrigpin, LOW);
        if (digitalRead(myechopin))
        {
            nbEcho++;
        }
    }
    /////////////////////////
    if (myServoOn == false)
    {
        digitalWrite(myservopin, HIGH);
        myServoOn = true;
    }
    //
    if (myServoOn == true && nbTick >= myServoTick)
    {
        digitalWrite(myservopin, LOW);
    }
}
//
int DiyBot::turretReadDistance_mm()
{
    int distance = 0;

    for (int i = 0; i < 4; i++)
    {
        distance += myTabValDistance[i];
    }
    //
    distance = distance / 4;
    distance = distance * 7;

    return distance;
}
//
void DiyBot::turretWriteAngle(int deg, int ms)
{
    if (deg > 180)
        deg = 180;
    //
    if (deg < 0)
        deg = 0;
    //
    this->myAngle = deg;
    myServoTick = (4 * this->myAngle) / 18;
    myServoTick += 15;
    //
    if (ms > 0)
        delay(ms);
}
//