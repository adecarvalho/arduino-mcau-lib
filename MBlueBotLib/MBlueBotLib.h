#ifndef MBLUEBLOTLIB_H
#define MBLUEBLOTLIB_H

#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "wiring_private.h"
#include "pins_arduino.h"
#ifndef F_CPU
#define  F_CPU 16000000UL
#endif
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>

//
typedef struct
{
  uint8_t s1;
  uint8_t s2;
} MePort_Sig;
//
//
struct cRGB {
  uint8_t g;
  uint8_t r;
  uint8_t b;
};

extern MePort_Sig mePort[11];//mePort[0] is nonsense

#define NC	-1
//
typedef enum
{
  PORT_0,
  PORT_1,
  PORT_2,
  PORT_3,
  PORT_4,
  PORT_5,
  PORT_6,
  PORT_7,
  PORT_8,
  M1,
  M2,
} MEPORT;
//
#define SLOT1 1
#define SLOT2 2
#define SLOT_1 SLOT1
#define SLOT_2 SLOT2

#define FALSE 0
#define TRUE  1

//
enum LineState {NO_LINE = -1, LINE_LEFT = 10, LINE_CENTER = 20, LINE_RIGHT = 30};
//*************************
//MePort
//
class MePort
{
  public:
    MePort();
    ///@brief initialize the Port
    ///@param port port number of device
    MePort(uint8_t port);
    MePort(uint8_t port, uint8_t slot);
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    uint8_t getPort();
    uint8_t getSlot();
    ///@return the level of pin 1 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead1();
    ///@return the level of pin 2 of port
    ///@retval true on HIGH.
    ///@retval false on LOW.
    bool dRead2();
    ///@brief set the analog value of pin 1 of port
    ///@param value is HIGH or LOW
    void dWrite1(bool value);
    ///@brief set the level of pin 1 of port
    ///@param value is HIGH or LOW
    void dWrite2(bool value);
    ///@return the analog signal of pin 1 of port between 0 to 1023
    int aRead1();
    ///@return the analog signal of pin 2 of port between 0 to 1023
    int aRead2();
    ///@brief set the PWM outpu value of pin 1 of port
    ///@param value between 0 to 255
    void aWrite1(int value);
    ///@brief set the PWM outpu value of pin 2 of port
    ///@param value between 0 to 255
    void aWrite2(int value);
    void reset(uint8_t port);
    void reset(uint8_t port, uint8_t slot);
    uint8_t pin1();
    uint8_t pin2();
    uint8_t pin();
    uint8_t pin(uint8_t port, uint8_t slot);
  protected:
    uint8_t s1;
    uint8_t s2;
    uint8_t _port;
    uint8_t _slot;
};
//*****************************************
//
//TWI
void i2c_init(void);
void waitTransmissionI2C();
void i2c_rep_start(uint8_t address);
void i2c_stop(void);
void i2c_write(uint8_t data );
uint8_t i2c_read(uint8_t ack);
uint8_t i2c_readAck();
uint8_t i2c_readNak(void);
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size);
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
int8_t i2c_readBit(uint8_t add, uint8_t reg, uint8_t bitNum);
int8_t i2c_readBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length);
void i2c_writeBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data);
//*********************************************
///@brief class of MeWire
class MeWire: public MePort
{
  public:
    MeWire(uint8_t address);
    ///@brief initialize
    ///@param port port number of device
    MeWire(MEPORT port, uint8_t address);
    ///@brief reset start index of i2c slave address.
    void setI2CBaseAddress(uint8_t baseAddress);
    bool isRunning();
    ///@brief Initiate the Wire library and join the I2C bus as a master or slave. This should normally be called only once.
    ///@param address the 7-bit slave address (optional); if not specified, join the bus as a master.
    void begin();
    ///@brief send one byte data request for read one byte from slave address.
    byte read(byte dataAddress);
    void read(byte dataAddress, uint8_t *buf, int len);
    ///@brief send one byte data request for write one byte to slave address.
    void write(byte dataAddress, byte data);
    void request(byte *writeData, byte *readData, int wlen, int rlen);
  protected:
    int _slaveAddress;
};
//
//***********************************************
//Motor
//
class MBlueMotor: public MePort
{
  public:
    MBlueMotor(MEPORT port);
    //
    void drive(int speed);
    //
    void stop(int pause_ms);
    //
    int getSpeed() const;

  protected:
    int mySpeed;
};
//*********************************************
//theMotors
class MBlueMotors
{
  public:
    MBlueMotors();
    //
    ~MBlueMotors();
    //
    void setSpeeds(int left, int right);
    //
    void setSpeeds(int left, int right, int ms);
    //
    void stop(int ms);
    //
    int getLeftSpeed() const;
    //
    int getRightSpeed() const;

  private:
    int mLeftSpeed;
    int mRightSpeed;
    
    MBlueMotor* mLeftMotor;
    MBlueMotor* mRightMotor;
};
//*********************************************
//btn A7
//
class MBlueButton
{
  public:
    MBlueButton(uint8_t pin = A7);

    //
    void waitForPressed();
    //
    void waitForReleased();
    //
    void waitForPressAndReleased();
    //
    bool isPressed();

  private:
    uint8_t myPin;
    int myVal;
};
//******************************************
//line Sensor
//
class MBlueLineSensor: public MePort
{
  public:
    MBlueLineSensor(MEPORT port);
    //
    bool readSensor1();
    //
    bool readSensor2();
    //
};
//
//***************************************
//Ultrasonic
//
class MBlueUltrasonic: public MePort
{
  public:
    MBlueUltrasonic(MEPORT port);
    //
    float getDictanceCm();
    //
    float getDictanceCm(uint16_t maxCm);

  protected:
    long mesure(unsigned long timeout);

};
//
//*****************************************
//led rgb
class MBlueRGBLed: public MePort {
  public:
    MBlueRGBLed();
    MBlueRGBLed(uint8_t pin);
    MBlueRGBLed(MEPORT port);
    MBlueRGBLed(MEPORT port, uint8_t slot);
    ~MBlueRGBLed();
    void reset(MEPORT port);
    void reset(MEPORT port, uint8_t slot);
    void reset(int pin);
    ///@brief set the count of leds.
    void setNumber(uint8_t num_led);
    ///@brief get the count of leds.
    uint8_t getNumber();
    ///@brief get the rgb value of the led with the index.
    cRGB getColorAt(uint8_t index);
    ///@brief set the rgb value of the led with the index.
    bool setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
    bool setColorAt(uint8_t index, long value);
    void setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
    void setColor(uint8_t red, uint8_t green, uint8_t blue);
    void setColor(long value);
    void clear();
    ///@brief become effective of all led's change.
    void show();

  private:
    uint16_t count_led;
    uint8_t *pixels;

    void rgbled_sendarray_mask(uint8_t *array, uint16_t length, uint8_t pinmask, uint8_t *port);

    const volatile uint8_t *ws2812_port;
    uint8_t pinMask;
};
//***********************************
//servo
class MBlueServo
{
  public:
  MBlueServo(MEPORT port=PORT4, uint8_t slot=1);

  begin();

  setAngle(int angleDeg);

  int getAngle() const;

  ~MBlueServo();

  protected:
  int mpin;
  int mservoangle;
  Servo* mservo;
};
////////////////////////////////////
#endif
