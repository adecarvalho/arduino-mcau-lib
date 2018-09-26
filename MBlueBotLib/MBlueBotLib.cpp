#include "MBlueBotLib.h"

MePort_Sig mePort[11] =
{
  { NC, NC }, { 11, 12 }, {  9, 10 }, { A2, A3 }, { A0, A1 },
  { NC, NC }, {  8, A6 }, { A7, 13 }, {  8, A6 }, {  6,  7 },
  {  5,  4 }
};

//**********************
/*        Port       */
//**********************
MePort::MePort() {
  s1 = mePort[0].s1;
  s2 = mePort[0].s2;
  _port = 0;
}
//*************************
MePort::MePort(uint8_t port)
{
  s1 = mePort[port].s1;
  s2 = mePort[port].s2;
  _port = port;
}
//**************************************
MePort::MePort(uint8_t port, uint8_t slot)
{
  s1 = mePort[port].s1;
  s2 = mePort[port].s2;
  _port = port;
  _slot = slot;
}
//************************
uint8_t MePort::getPort() {
  return _port;
}
//************************
uint8_t MePort::getSlot() {
  return _slot;
}
//*******************
bool MePort::dRead1()
{
  bool val;
  pinMode(s1, INPUT);
  val = digitalRead(s1);
  return val;
}
//*******************
bool MePort::dRead2()
{
  bool val;
  pinMode(s2, INPUT);
  val = digitalRead(s2);
  return val;
}
//******************************
void MePort::dWrite1(bool value)
{
  pinMode(s1, OUTPUT);
  digitalWrite(s1, value);
}
//******************************
void MePort::dWrite2(bool value)
{
  pinMode(s2, OUTPUT);
  digitalWrite(s2, value);
}
//******************
int MePort::aRead1()
{
  int val;
  val = analogRead(s1);
  return val;
}
//******************
int MePort::aRead2()
{
  int val;
  val = analogRead(s2);
  return val;
}
//*****************************
void MePort::aWrite1(int value)
{
  analogWrite(s1, value);
}
//*****************************
void MePort::aWrite2(int value)
{
  analogWrite(s2, value);
}
//********************************
void MePort::reset(uint8_t port) {
  s1 = mePort[port].s1;
  s2 = mePort[port].s2;
  _port = port;
}
//*******************************************
void MePort::reset(uint8_t port, uint8_t slot) {
  s1 = mePort[port].s1;
  s2 = mePort[port].s2;
  _port = port;
  _slot = slot;
}
//*********************
uint8_t MePort::pin1() {
  return s1;
}
//**********************
uint8_t MePort::pin2() {
  return s2;
}
//********************
uint8_t MePort::pin() {
  return _slot == SLOT1 ? s1 : s2;
}
//*********************************************
uint8_t MePort::pin(uint8_t port, uint8_t slot) {
  return slot == SLOT1 ? mePort[port].s1 : mePort[port].s2;
}
//*****************************************************
//*****************************************************
/* I2C */
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;
//*******************
void i2c_init(void) {

  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / 1000000) - 16) / 2;       // change the I2C clock rate
  TWCR = 1 << TWEN;                            // enable twi module, no interrupt
}
//**************************
void waitTransmissionI2C() {
  uint16_t count = 512; // change to 512 for lego encoder motor, the timer may overflow when rep-start
  while (!(TWCR & (1 << TWINT))) {
    count--;
    if (count == 0) {            //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      Serial.println("i2cerr");
      break;
    }
  }
}
//************************************
void i2c_rep_start(uint8_t address) {
  TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1 << TWINT) | (1 << TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}
//******************
void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}
//*****************************
void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1 << TWINT) | (1 << TWEN);
  waitTransmissionI2C();
}
//*****************************
uint8_t i2c_read(uint8_t ack) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}
//*********************
uint8_t i2c_readAck() {
  return i2c_read(1);
}
//*************************
uint8_t i2c_readNak(void) {
  return i2c_read(0);
}
//***********************************************************
size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add << 1) | 1); // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = i2c_read(size > 0);
    /* TODO catch I2C errors here and abort */
    bytes_read++;
  }
  return bytes_read;
}
//****************************************************************************
size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}
//********************************************************
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add << 1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}
//*********************************************
uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}
//************************************************************
int8_t i2c_readBit(uint8_t add, uint8_t reg, uint8_t bitNum) {
  uint8_t b;
  b = i2c_readReg(add, reg);
  return b & (1 << bitNum);
}
//*******************************************************************************
int8_t i2c_readBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length) {
  uint8_t b;
  b = i2c_readReg(dev, reg);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  b &= mask;
  b >>= (bitStart - length + 1);
  return b;
}
//**********************************************************************************
void i2c_writeBits(uint8_t dev, uint8_t reg, uint8_t bitStart, uint8_t length, uint8_t data) {
  uint8_t b;
  b = i2c_readReg(dev, reg);
  uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1); // shift data into correct position
  data &= mask; // zero all non-important bits in data
  b &= ~(mask); // zero all important bits in existing byte
  b |= data; // combine data with existing byte
  i2c_writeReg(dev, reg, b);
}
//
//Wire Setup
#define BEGIN_FLAG          0x1E
#define BEGIN_STATE         0x91

//**************************************
/*             Wire               */
MeWire::MeWire(uint8_t address): MePort()
{
  _slaveAddress = address + 1;
}
//********************************************************
MeWire::MeWire(MEPORT port, uint8_t address): MePort(port)
{
  _slaveAddress = address + 1;
}
//******************
void MeWire::begin()
{
  delay(1000);
  Wire.begin();
  write(BEGIN_FLAG, 0x01);
}
//*********************
bool MeWire::isRunning()
{
  return read(BEGIN_STATE);
}
//************************************************
void MeWire::setI2CBaseAddress(uint8_t baseAddress)
{
  byte w[2] = {0};
  byte r[4] = {0};
  w[0] = 0x21;
  w[1] = baseAddress;
  request(w, r, 2, 4);
}
//*********************************
byte MeWire::read(byte dataAddress)
{
  byte *b = {0};
  read(dataAddress, b, 1);
  return b[0];
}
//*******************************************************
void MeWire::read(byte dataAddress, uint8_t *buf, int len)
{
  byte rxByte;
  Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(dataAddress); // sends one byte
  Wire.endTransmission(); // stop transmitting
  // delayMicroseconds(1);
  delay(100);
  Wire.requestFrom(_slaveAddress, len); // request 6 bytes from slave device
  int index = 0;
  while (Wire.available()) // slave may send less than requested
  {
    rxByte = Wire.read(); // receive a byte as character
    buf[index] = rxByte;
    index++;
  }
}
//*********************************************
void MeWire::write(byte dataAddress, byte data)
{
  Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(dataAddress); // sends one byte
  // Wire.endTransmission(); // stop transmitting

  // Wire.beginTransmission(_slaveAddress); // transmit to device
  Wire.write(data); // sends one byte
  Wire.endTransmission(); // stop transmitting
}
//***********************************************************************
void MeWire::request(byte *writeData, byte *readData, int wlen, int rlen)
{
  uint8_t rxByte;
  uint8_t index = 0;
  delay(10);
  Wire.beginTransmission(_slaveAddress); // transmit to device

  Wire.write(writeData, wlen);

  Wire.endTransmission();//delay(10);
  delayMicroseconds(2);
  Wire.requestFrom(_slaveAddress, rlen); // request 6 bytes from slave device
  // delayMicroseconds(2);
  while (Wire.available()) // slave may send less than requested
  {
    rxByte = Wire.read(); // receive a byte as character

    readData[index] = rxByte;
    index++;
  }
}
//****************************************
//BlueMotor
//
MBlueMotor::MBlueMotor(MEPORT port): MePort(port)
{
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);

  mySpeed = 0;
}
//
void MBlueMotor::drive(int speed)
{
  //limite
  if (speed >= 255)
    speed = 255;

  //
  if (speed < -255)
    speed = -255;
  //
  mySpeed = speed;
  //
  if (mySpeed >= 0)
  {
    MePort::dWrite2(HIGH);
    MePort::aWrite1(mySpeed);
  }
  else
  {
    MePort::dWrite2(LOW);
    MePort::aWrite1(-mySpeed);
  }
}
//**********************
void MBlueMotor::stop(int pause_ms)
{
  MBlueMotor::drive(0);
  mySpeed = 0;
  //
  if (pause_ms > 0)
  {
    delay(pause_ms);
  }
}
//*************************
int MBlueMotor::getSpeed() const {
  return mySpeed;
}
//******************************************
//button A7
//
MBlueButton::MBlueButton(uint8_t pin)
{
  myPin = pin;
  myVal = 1024;
}
//******************************
bool MBlueButton::isPressed()
{
  myVal = analogRead(myPin);

  return (myVal <= 100);
}
//*******************************
void MBlueButton::waitForReleased()
{
  do
  {
    while (isPressed());
    delay(10);
  }
  while (isPressed());
}
//*******************************
void MBlueButton::waitForPressAndReleased()
{
  MBlueButton::waitForPressed();
  MBlueButton::waitForReleased();
}
//*******************************
void MBlueButton::waitForPressed()
{
  do
  {
    while (! isPressed());
    delay(10);
  }
  while (!isPressed());
}
//****************************
//line sensor
//*****************************
MBlueLineSensor::MBlueLineSensor(MEPORT port): MePort(port)
{
}
//*****************************
bool MBlueLineSensor::readSensor1()
{
  return MePort::dRead1();
}
//******************************
bool MBlueLineSensor::readSensor2()
{
  return MePort::dRead2();
}
//
//*****************************
//ultrasonic
MBlueUltrasonic::MBlueUltrasonic(MEPORT port): MePort(port)
{
}
//******************************
float MBlueUltrasonic::getDictanceCm()
{
  return MBlueUltrasonic::getDictanceCm(100);
}
//*********************************
float MBlueUltrasonic::getDictanceCm(uint16_t maxCm)
{
  long distance = MBlueUltrasonic::mesure(maxCm * 55 + 200);

  return (float)(distance / 58.0);
}
//********************************
long MBlueUltrasonic::mesure(unsigned long timeout)
{
  long duration = 5800;
  MePort::dWrite2(LOW);
  delayMicroseconds(2);
  //
  MePort::dWrite2(HIGH);
  delayMicroseconds(10);
  MePort::dWrite2(LOW);
  //
  pinMode(s2, INPUT);
  duration = pulseIn(s2, HIGH, timeout);

  if (duration < 1)
    duration = 5800;

  return duration;

}
//*******************************
//led rgb
//
MBlueRGBLed::MBlueRGBLed()
{
  pinMask = digitalPinToBitMask(13);
  ws2812_port = portOutputRegister(digitalPinToPort(13));
  pinMode(13, OUTPUT);
  //setNumber(4);
  setNumber(2);
}
//****************************
MBlueRGBLed::MBlueRGBLed(uint8_t pin)
{
  s2 = pin;
  pinMask = digitalPinToBitMask(pin);
  ws2812_port = portOutputRegister(digitalPinToPort(pin));
  pinMode(pin, OUTPUT);
  setNumber(2);
}
//******************************************
MBlueRGBLed::MBlueRGBLed(MEPORT port): MePort(port)
{
  pinMask = digitalPinToBitMask(s2);
  ws2812_port = portOutputRegister(digitalPinToPort(s2));
  pinMode(s2, OUTPUT);
  setNumber(2);
}
//********************************************************
MBlueRGBLed::MBlueRGBLed(MEPORT port, uint8_t slot): MePort(port)
{
  if (slot == SLOT2)
  {
    pinMask = digitalPinToBitMask(s2);
    ws2812_port = portOutputRegister(digitalPinToPort(s2));
    pinMode(s2, OUTPUT);
  }
  else
  {
    pinMask = digitalPinToBitMask(s1);
    ws2812_port = portOutputRegister(digitalPinToPort(s1));
    pinMode(s1, OUTPUT);
  }
  setNumber(2);
}
//*****************************
void MBlueRGBLed::reset(int pin) {
  pinMask = digitalPinToBitMask(pin);
  ws2812_port = portOutputRegister(digitalPinToPort(pin));
  pinMode(pin, OUTPUT);
}
//*******************************
void MBlueRGBLed::reset(MEPORT port)
{
  s2 = mePort[port].s2;
  s1 = mePort[port].s1;
  pinMask = digitalPinToBitMask(s2);
  ws2812_port = portOutputRegister(digitalPinToPort(s2));
  pinMode(s2, OUTPUT);
}
//*********************************************
void MBlueRGBLed::reset(MEPORT port, uint8_t slot)
{
  s2 = mePort[port].s2;
  s1 = mePort[port].s1;
  if (slot == SLOT2)
  {
    pinMask = digitalPinToBitMask(s2);
    ws2812_port = portOutputRegister(digitalPinToPort(s2));
    pinMode(s2, OUTPUT);
  }
  else
  {
    pinMask = digitalPinToBitMask(s1);
    ws2812_port = portOutputRegister(digitalPinToPort(s1));
    pinMode(s1, OUTPUT);
  }
  setNumber(4);
}
//****************************************
void MBlueRGBLed::setNumber(uint8_t num_leds)
{
  count_led = num_leds;
  pixels = (uint8_t *)malloc(count_led * 3);
  clear();
}
//*************************************
cRGB MBlueRGBLed::getColorAt(uint8_t index)
{

  cRGB px_value;

  if (index < count_led)
  {

    uint8_t tmp;
    tmp = index * 3;

    px_value.g = pixels[tmp];
    px_value.r = pixels[tmp + 1];
    px_value.b = pixels[tmp + 2];
  }

  return px_value;
}
//***************************
uint8_t MBlueRGBLed::getNumber()
{
  return count_led;
}
//********************************************************************************
bool MBlueRGBLed::setColorAt(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
  if (index < count_led)
  {
    uint8_t tmp = index * 3;
    pixels[tmp] = green;
    pixels[tmp + 1] = red;
    pixels[tmp + 2] = blue;

    return true;
  }
  return false;
}
//*************************************************
bool MBlueRGBLed::setColorAt(uint8_t index, long value)
{
  if (index < count_led)
  {
    uint8_t tmp = index * 3;
    uint8_t red = (value & 0xff0000) >> 16;
    uint8_t green = (value & 0xff00) >> 8;
    uint8_t blue = value & 0xff;
    pixels[tmp] = green;
    pixels[tmp + 1] = red;
    pixels[tmp + 2] = blue;
    return true;
  }
  return false;
}
//********************************************************************************
void MBlueRGBLed::setColor(uint8_t index, uint8_t red, uint8_t green, uint8_t blue) {
  if (index == 0) {
    setColor(red, green, blue);
  } else {
    setColorAt(index - 1, red, green, blue);
    show();
  }
}
//**************************************************************
void MBlueRGBLed::setColor(uint8_t red, uint8_t green, uint8_t blue)
{
  for (int i = 0; i < count_led; ++i)
  {
    uint8_t tmp = i * 3;
    pixels[tmp] = green;
    pixels[tmp + 1] = red;
    pixels[tmp + 2] = blue;
  }
  show();
}
//********************************
void MBlueRGBLed::setColor(long value)
{
  for (int i = 0; i < count_led; ++i)
  {
    uint8_t tmp = i * 3;
    uint8_t red = (value & 0xff0000) >> 16;
    uint8_t green = (value & 0xff00) >> 8;
    uint8_t blue = value & 0xff;
    pixels[tmp] = green;
    pixels[tmp + 1] = red;
    pixels[tmp + 2] = blue;
  }
  show();
}
//*************************************
void MBlueRGBLed::clear()
{
  // for(int i = 0; i < count_led; i++)
  // {
  //     setColorAt(i, 0, 0, 0);
  // }
  memset(pixels, 0, 3 * count_led);
  show();
}
/*
  This routine writes an array of bytes with RGB values to the Dataout pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    3
#define w_fixedhigh   6
#define w_fixedtotal  10

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
#define w1_nops w1
#else
#define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
#error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
#warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
#warning "Please consider a higher clockspeed, if possible"
#endif

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8
//*****************************************************************************
void  MBlueRGBLed::rgbled_sendarray_mask(uint8_t *data, uint16_t datlen, uint8_t maskhi, uint8_t *port)
{
  uint8_t curbyte, ctr, masklo;
  uint8_t oldSREG = SREG;
  cli();  //Disables all interrupts

  masklo = *port & ~maskhi;
  maskhi = *port | maskhi;

  while (datlen--)
  {
    curbyte = *data++;

    asm volatile(
      "       ldi   %0,8  \n\t"
      "loop%=:            \n\t"
      "       st    X,%3 \n\t"    //  '1' [02] '0' [02] - re
#if (w1_nops&1)
      w_nop1
#endif
#if (w1_nops&2)
      w_nop2
#endif
#if (w1_nops&4)
      w_nop4
#endif
#if (w1_nops&8)
      w_nop8
#endif
#if (w1_nops&16)
      w_nop16
#endif
      "       sbrs  %1,7  \n\t"    //  '1' [04] '0' [03]
      "       st    X,%4 \n\t"     //  '1' [--] '0' [05] - fe-low
      "       lsl   %1    \n\t"    //  '1' [05] '0' [06]
#if (w2_nops&1)
      w_nop1
#endif
#if (w2_nops&2)
      w_nop2
#endif
#if (w2_nops&4)
      w_nop4
#endif
#if (w2_nops&8)
      w_nop8
#endif
#if (w2_nops&16)
      w_nop16
#endif
      "       brcc skipone%= \n\t"    //  '1' [+1] '0' [+2] -
      "       st   X,%4      \n\t"    //  '1' [+3] '0' [--] - fe-high
      "skipone%=:               "     //  '1' [+3] '0' [+2] -

#if (w3_nops&1)
      w_nop1
#endif
#if (w3_nops&2)
      w_nop2
#endif
#if (w3_nops&4)
      w_nop4
#endif
#if (w3_nops&8)
      w_nop8
#endif
#if (w3_nops&16)
      w_nop16
#endif

      "       dec   %0    \n\t"    //  '1' [+4] '0' [+3]
      "       brne  loop%=\n\t"    //  '1' [+5] '0' [+4]
      :   "=&d" (ctr)
      //    : "r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (maskhi), "r" (masklo)
      :   "r" (curbyte), "x" (port), "r" (maskhi), "r" (masklo)
    );
  }

  SREG = oldSREG;
}
//**********************
void MBlueRGBLed::show()
{
  //  *ws2812_port_reg |= pinMask; // Enable DDR
  rgbled_sendarray_mask(pixels, 3 * count_led, pinMask, (uint8_t *) ws2812_port);
}
//***************************
MBlueRGBLed::~MBlueRGBLed()
{
}
/////////////////////////////////////
//**********************************
MBlueMotors::MBlueMotors()
{
  mLeftSpeed = 0;
  mRightSpeed = 0;

  mLeftMotor = new MBlueMotor(M1);
  mRightMotor = new MBlueMotor(M2);
}
//**********************************
MBlueMotors::~MBlueMotors()
{
  delete(mLeftMotor);
  delete(mRightMotor);
}
//**********************************
void MBlueMotors::setSpeeds(int left, int right)
{
  this->mLeftMotor->drive(-left);
  this->mLeftSpeed = left;
  //
  this->mRightMotor->drive(right);
  this->mRightSpeed = right;
}
//******************************************
void MBlueMotors::setSpeeds(int left, int right, int ms)
{
  MBlueMotors::setSpeeds(left, right);

  if (ms > 0)
  {
    delay(ms);
    MBlueMotors::setSpeeds(0, 0);
  }
}
//********************************
void MBlueMotors::stop(int ms)
{
  MBlueMotors::setSpeeds(0, 0);
  //
  if (ms > 0)
  {
    delay(ms);
  }
}
//************************************
int MBlueMotors::getLeftSpeed() const
{
  return  this->mLeftSpeed;
}
//**********************************
int MBlueMotors::getRightSpeed() const
{
  return this->mRightSpeed;
}
//*******************************
// servo
//*******************************
//
MBlueServo:: MBlueServo(MEPORT port, uint8_t slot)
{
  if(slot==1)
  {
    this->mpin=mePort[port].s1;
  }
  else
  {
    this->mpin=mePort[PORT4].s2;
  }
  //

  this->mservo= new Servo();

}
//*********************************
MBlueServo::begin()
{
  this->mservo->attach(this->mpin);
  delay(200);
  this->mservo->write(85);
}
//*********************************
MBlueServo::setAngle(int angleDeg)
{
   mservoangle=angleDeg;
   
  //max min
  if( mservoangle >= 160)
  {
    mservoangle=160;
  }
  //
  if(mservoangle <20)
  {
    mservoangle=20;
  }
  //
  //
  this->mservo->write(mservoangle);
}
//*******************************
int MBlueServo::getAngle() const
{
  return this->mservoangle;
}
//********************************
MBlueServo::~MBlueServo()
{
  delete(this->mservo);
}
////////////////////////////////

