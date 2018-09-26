#include "ChipLib3.h"

/////////////////////////////

DigitalOut::DigitalOut(uint8_t pin):myPin(pin){
  this->myValue=0;
  this->myBit=0;
  this->myPort=0;

}
//--------------------
void DigitalOut::begin(void){
 
  this->myBit=digitalPinToBitMask(this->myPin);
  this->myPort=digitalPinToPort(this->myPin);

  volatile uint8_t *myDdr=portModeRegister(this->myPort);

  myOutput=portOutputRegister(this->myPort);
  
  *myDdr|= this->myBit;	//bit output =1
  *myOutput &= ~this->myBit;//bit =0

}
//--------------------
DigitalOut:: ~DigitalOut(void){
}
//------------------------------
void DigitalOut::write(uint8_t value){

  this->myValue=value;

  if(myValue){
  *myOutput |= this->myBit;//bit =1
  }
 //
  else {
  *myOutput &= ~this->myBit;//bit =0
  }
}
//---------------------------
uint8_t DigitalOut::read(void) const{
  return this->myValue;
}
//---------------------------
void DigitalOut::operator=(uint8_t value){
  write(value);
}
//-----------------------
//-----------------------
DigitalIn::DigitalIn(uint8_t pin):
myPin(pin){
  this->myValue=0;
}
//--------------------------------
DigitalIn::~DigitalIn(void){
}
//----------------------------------
void DigitalIn::begin(void){
  pinMode(this->myPin,INPUT);
}
//-----------------------------
uint8_t DigitalIn::read(void){
  this->myValue=digitalRead(this->myPin);
  return this->myValue;
}
//-----------------------------
//-----------------------------
AnalogIn::AnalogIn(uint8_t pin):
myIn(pin){
  this->myVal=0;
}
//--------------------------------
AnalogIn::~AnalogIn(void){
}
//-----------------------------------
void AnalogIn::begin(void){

  pinMode(myIn,INPUT);

  ADCSRA &=~PS_128;

  ADCSRA |=PS_16; //16MH/16=1MHz soit Test Tconv=20us
}
//--------------------------------
int16_t AnalogIn::read(void){
  myVal=analogRead(myIn);
  return myVal;
}
//------------------------------
//------------------------------
AnalogOut::AnalogOut(uint8_t pinSS,uint8_t pinLdac,uint8_t clock_div):
mySSPin(pinSS),myLDACPin(pinLdac),myClockDiv(clock_div){
  this->myValue=0;
}
//-------------------------------
void AnalogOut::begin(void){
  SPI.begin();

  pinMode(mySSPin,OUTPUT);
  digitalWrite(mySSPin,HIGH);


  pinMode(myLDACPin,OUTPUT);
  digitalWrite(myLDACPin,HIGH);

  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(myClockDiv);
}
//--------------------------
void AnalogOut::write(int Nvalue){

  //test
  if(Nvalue >1023)
    Nvalue=1023;

  if(Nvalue <0)
    Nvalue=0;

  //
  myValue=Nvalue;
  myValue=myValue<<2;

  digitalWrite(mySSPin,LOW);

  uint16_t data=0x7000;

  data=data|myValue;

  SPI.transfer((data & 0xFF00)>>8);
  SPI.transfer(data & 0xFF);

  digitalWrite(mySSPin,HIGH);

  delayMicroseconds(1);

  digitalWrite(myLDACPin,LOW);

  delayMicroseconds(1);

  digitalWrite(myLDACPin,HIGH);
}
//-------------------------
AnalogOut::~AnalogOut(void){
}
//-----------------------------
//------------------------------
ButtonIn::ButtonIn(uint8_t pin,uint8_t pull):
myPin(pin),myPull(pull){

  this->myState=HIGH;
}
//-------------------------------
void ButtonIn::begin(void){

  if(myPull==PULL_UP_ENABLE)
    pinMode(myPin,INPUT_PULLUP);

  else
    pinMode(myPin,INPUT);

  delayMicroseconds(5);
}
//--------------------------------
void ButtonIn::waitForPress(){

  do
  {
    while(! isPressed());   //attendre action sur button
    delay(10);
  }
  while(!isPressed());
  myState=LOW;
}
//---------------------------------
void ButtonIn::waitForRelease(){
  do
  {
    while(isPressed());   //attendre action sur button
    delay(10);
  }
  while(isPressed());
  myState=HIGH;
}
//--------------------------------
void ButtonIn::waitForPressAndRelease(){
  waitForPress();
  waitForRelease();
}
//--------------------------------
boolean ButtonIn::isPressed(){
  return(digitalRead(myPin)==LOW);
}
//-------------------------------
ButtonIn::~ButtonIn(){
}
//-----------
// SoftTimer 
//-------------------------
SoftTimer::SoftTimer(void) {
  this->myLastTime=0;
  this->myPeriod=0;
  this->isAction=false;
  this->myCallback=0;
}
//---------------------------------------------
void SoftTimer::update(void) {

  unsigned long now=millis();
  if(isAction)
  {
    if((now-myLastTime)>=myPeriod)
    {
      this->myLastTime=now;
      (*myCallback)();
    }
  }
}
//----------------------------------------------
void SoftTimer::attach(void (*callback)(void),unsigned long periodMs){

  myPeriod=periodMs;
  this->myCallback=callback;
}
//---------------------------------------------
void SoftTimer::start(void){
  this->isAction=true;
  this->myLastTime=0;
}
//---------------------------------------------
void SoftTimer::stop(void) {
  this->isAction=false;
}
//---------------------------------------------
SoftTimer::~SoftTimer(void){
}
//---------------
// Note for sound
//--------------------------------------------------------------
Note::Note(char _note,bool _isSharp,char _octave,int _duration):
myNoteChar(_note),myIsSharp(_isSharp),myOctave(_octave),myDuration(_duration){

  myFrequency=calculateFreq();
}
//-------------------------------------
Note::Note(const char* _str,int _duration):
myDuration(_duration){

  parse(_str);
  myFrequency=calculateFreq();
}
//-------------------------------------
void Note::parse(const char* _str){

  char i=0;

  for(i=0; i<3; i++)
  {
    switch(_str[i])
    {
      //
    case 'R':
      myNoteChar='R';
      break;
      //
    case 'r':
      myNoteChar='R';
      break;
      //
    case 'A':
      myNoteChar='A';
      break;
      //
    case 'a':
      myNoteChar='A';
      break;
      //
    case 'B':
      myNoteChar='B';
      break;
      //
    case 'b':
      myNoteChar='B';
      break;
      //
    case 'C':
      myNoteChar='C';
      break;
      //
    case 'c':
      myNoteChar='C';
      break;
      //
    case 'D':
      myNoteChar='D';
      break;
      //
    case 'd':
      myNoteChar='D';
      break;
      //
    case 'E':
      myNoteChar='E';
      break;
      //
    case 'e':
      myNoteChar='E';
      break;
      //
    case 'F':
      myNoteChar='F';
      break;
      //
    case 'f':
      myNoteChar='F';
      break;
      //
    case 'G':
      myNoteChar='G';
      break;
      //
    case 'g':
      myNoteChar='G';
      break;
      //
    case '%':
      myIsSharp=false;
      break;
      //
    case '#':
      myIsSharp=true;
      break;
      //
    case '3':
      myOctave=3;
      break;
      //
    case '4':
      myOctave=4;
      break;
      //
    case '5':
      myOctave=5;
      break;
      //
    case '6':
      myOctave=6;
      break;
      //
    case '7':
      myOctave=7;
      break;
      //
    default:
      myNoteChar='R';
      break;
    }
  }
}
//-------------------------------------
int Note::calculateFreq(void){

  switch(myNoteChar)
  {
  case 'p':
    return NOTE_R;
    //
  case 'R':
    return NOTE_R;
    //
  case 'r':
    return NOTE_R;
    //
  case 'A':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_A[myOctave-3];
      else
        return tab_AS[myOctave-3];
    }
    break;
    //
  case 'a':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_A[myOctave-3];
      else
        return tab_AS[myOctave-3];
    }
    break;
    //
  case 'B':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_B[myOctave-3];
      else
        return tab_BS[myOctave-3];
    }
    break;
    //
  case 'b':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_B[myOctave-3];
      else
        return tab_BS[myOctave-3];
    }
    break;
    //
  case 'C':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_C[myOctave-3];
      else
        return tab_CS[myOctave-3];
    }
    break;
    //
  case 'c':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_C[myOctave-3];
      else
        return tab_CS[myOctave-3];
    }
    break;
    //
  case 'D':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_D[myOctave-3];
      else
        return tab_DS[myOctave-3];
    }
    break;
    //
  case 'd':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_D[myOctave-3];
      else
        return tab_DS[myOctave-3];
    }
    break;
    //
  case 'E':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_E[myOctave-3];
      else
        return tab_ES[myOctave-3];
    }
    break;
    //
  case 'e':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_E[myOctave-3];
      else
        return tab_ES[myOctave-3];
    }
    break;
    //
  case 'F':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_F[myOctave-3];
      else
        return tab_FS[myOctave-3];
    }
    break;
    //
  case 'f':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_F[myOctave-3];
      else
        return tab_FS[myOctave-3];
    }
    break;
    //
  case 'G':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_G[myOctave-3];
      else
        return tab_GS[myOctave-3];
    }
    break;
    //
  case 'g':
    if(myOctave >=3 && myOctave <=7)
    {
      if(myIsSharp==false)
        return tab_G[myOctave-3];
      else
        return tab_GS[myOctave-3];
    }
    break;
    //
  default:

    break;
  }
}
//---------------------------
int Note::getNoteFreq(void)const{
  return this->myFrequency;
}
//------------------------------
char Note::getNoteChar(void)const{
  return this->myNoteChar;
}
//-------------------------------
char Note::getNoteOctave(void)const{
  return this->myOctave;
}
//--------------------------------
int Note::getNoteDuration(void)const{
  return this->myDuration;
}
//--------------------------------------
Note::~Note(){}
//-------------------------------------
// device Buzzer for tone sounds and music
//-------------------------------------
Buzzer::Buzzer(char _pin):
myPin(_pin) {
  myIsMusicFinish=false;
}
//--------------------------------------
Buzzer:: ~Buzzer(){}
//--------------------------------------
void Buzzer::begin(void){
  pinMode(myPin,OUTPUT);

  digitalWrite(myPin,LOW);
}
//--------------------------------------
void Buzzer::sound(const Note* _note) {

  int freq = _note->getNoteFreq();
  int dur = _note->getNoteDuration();
  
  unsigned long peri=1000000/freq;
  unsigned long len=0;
  boolean state=false;
  
  for(len=0;len < (long) dur*1000; len+=peri)
  {
    state=!state;
    digitalWrite(myPin,state);
    delayMicroseconds(peri-50);
  }
  //tone(myPin,freq,dur);

  delay(dur);

  digitalWrite(myPin,LOW);
}

//--------------------------------------
void Buzzer::music(const char* p) {

  myIsMusicFinish=false;
  
  static Note* theNote=NULL;

  char defautDuree=4;
  char defautOctave=6;
  int bmp=63;
  int ronde=0;
  int dura=0;
  int num=0;

  char _note='a';
  bool _issharp=false;
  char _octave=3;
  char newoctave=3;
  //
  while(*p != ':')
    p++;   //le titre
  p++;

  //defaut duration
  if(*p=='d')
  {
    p++;
    p++;    //skip d=
    while(isDigit(*p))
    {
      num=(num*10)+(*p++ -'0');
    }
    //
    if(num>0)
      defautDuree=num;
    p++;    //skip ,
  }
  //
  if(*p=='o')
  {
    p++;
    p++;    //skip o=
    num=*p++ - '0';

    if(num>=3 && num <=7)
      defautOctave=num;
    p++;    //skip ,
  }
  //
  num=0;
  if(*p == 'b')
  {
    p++;
    p++;  //skip b=
    while(isDigit(*p))
    {
      num=num*10+(*p++ - '0');
    }
    bmp=num;
    p++;  //skip:
  }
  //
  ronde=(60*500L/bmp)*4;   //in ms
  //les notes
  while(*p)
  {
    //get Duration if available
    num=0;
    while(isDigit(*p))
    {
      num=(num*10)+(*p++ -'0');
    }
    if(num)
      dura=ronde/num;
    else
      dura=ronde/defautDuree;
    //la note
    _note=*p;
    p++;
    //issharp
    if(*p=='#')
    {
      _issharp=true;
      p++;
    }
    else
    {
      _issharp=false;
    }
    //un point . ?
    if(*p=='.')
    {
      dura+=dura/2;
      p++;
    }
    //nouvelle octave ?
    if(isDigit(*p))
    {
      newoctave=*p - '0';
      p++;
    }
    if(newoctave >0)
      _octave=newoctave;

    else
      _octave=defautOctave;

    //
    if(*p==',')
    {
      p++;
      theNote= new Note(_note,_issharp,_octave,dura);
      sound(theNote);
      // theNote->toString();
    }
  }
  delete(theNote);
  myIsMusicFinish=true;
}
//-----------------------------------------------
boolean Buzzer::isMusicFinish(void) const
{
  return myIsMusicFinish;
}
//--------------------------------------------
















