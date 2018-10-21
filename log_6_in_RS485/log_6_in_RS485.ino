//<debuuging defines>
#define   _DEBUG_ // for serial debugging
#undef    _DEBUG_
#define   _DEBUGM_ // to print millis() to SD card
#undef    _DEBUGM_
#ifdef  _DEBUG_
#define _DEBUGM_
#endif
//</debuging defines>
//<pin defines>
#define B1OXYGEN        A0
#define B2OXYGEN        A1
#define TPOS            A2
#define ERRORLED_RED    5 // pwm mode
#define ERRORLED_GREEN  6 // pwm mode
#define HEAT_1          7 // digital input
#define HEAT_2          8 // digital input
//</pin defines>
//<led defines>
#define OFF             LOW
#define RED_ON          32
#define GREEN_ON        127
//</led defines>

#include <SPI.h>
#include <SD.h>
unsigned long startmillis = 0;
unsigned long loopcount = 0;


long mymap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

void setup() {
  // put your setup code here, to run once:
  //
  // <set all unused pins to output and LOW>  
  // digital pins
  int i;
  const int numunused =7;  
  uint8_t unused_pins[numunused]{2,3,4,9,17,18,19};
    for(i=0;i<numunused;i++){
      digitalWrite(unused_pins[i],LOW);
      pinMode(unused_pins[i],OUTPUT);
      }
  // </set all unused pins to output and LOW>

  // <digital pins in use>
    pinMode(HEAT_1,INPUT);
    pinMode(HEAT_2,INPUT);
  // </digital pins in use>
  
  analogWrite(ERRORLED_RED,OFF);
  //pinMode (ERRORLED_RED,OUTPUT);
  analogWrite(ERRORLED_GREEN,OFF);
  //pinMode (ERRORLED_GREEN,OUTPUT);

  

  // disable input buffers on ADC pins per datasheet
  DIDR0 = 0x7E; //adc0 through 6
  //delay(20); // a short delay to let things stabilize
  #ifdef  _DEBUG_
  Serial.begin(57600);
  Serial.print("Initializing SD card...");
  #endif
  // see if the card is present and can be initialized:
  if (!SD.begin(10,11,12,13)) {
    #ifdef  _DEBUG_
    Serial.println("Card failed, or not present");
    // don't do anything more:
    #endif
    digitalWrite(ERRORLED_GREEN,OFF); // no card inserted
    digitalWrite(ERRORLED_RED,RED_ON);
    while (1);
    
  }
  #ifdef _DEBUG_
  Serial.println("card initialized.");
  #endif
  digitalWrite(ERRORLED_RED,OFF); // all is well, get on with it
  digitalWrite(ERRORLED_GREEN,GREEN_ON);
  startmillis = millis();
}


void loop() {
  // put your main code here, to run repeatedly:
int trash, tpos, b1oxygen, b2oxygen,temp;

String dataString = "";

if (loopcount == 0){
dataString += String(millis()-startmillis);
dataString += String(" : ");
}

// how many times have we done this
loopcount++;
if(loopcount >= 1000){
  loopcount =0;
  } 

#ifdef _DEBUGM_
dataString += String(millis()-startmillis);
dataString += String(" : ");
#endif

  analogReference(DEFAULT);       // use Vcc reference for tpos
  delayMicroseconds(250);
  // <get us some throttle info> 
  // arduino reference says first reading after referenc change are junk 
  trash = analogRead(TPOS);
  trash = analogRead(TPOS);
  trash = analogRead(TPOS);
  // should be good to get accurate readings now
  temp = analogRead(TPOS);
  tpos = mymap(temp,0,1023,0,5000);
  dataString += String(tpos);
  dataString += String(",");
  // </get us some throttle info>
  // <get us some heater info>
  dataString += String(digitalRead(HEAT_1));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_2));
  dataString += String(",");
  
  analogReference(INTERNAL);      // use 1.1v reference for O2 sensors
  delayMicroseconds(250);
  trash = analogRead(B1OXYGEN);
  trash = analogRead(B1OXYGEN);
  trash = analogRead(B1OXYGEN);  
  temp = analogRead(B1OXYGEN);
  b1oxygen = mymap(temp,0,1023,0,1100);
  temp = analogRead(B2OXYGEN);
  b2oxygen = mymap(temp,0,1023,0,1100);
  dataString += String(b1oxygen);
  dataString += String(",");
  dataString += String(b2oxygen);

  // <SD card setup>
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
     #ifdef _DEBUG_
    // print to the serial port too:
    Serial.println(dataString);
     #endif
  }
  // if the file isn't open, pop up an error:
  else {
    #ifdef _DEBUG_
    Serial.println("error opening datalog.txt");
    #endif
    analogWrite (ERRORLED_GREEN,GREEN_ON); // yellow if file error
    analogWrite (ERRORLED_RED,RED_ON);
  }
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ enabled
  
  #ifdef  _DEBUG_
    delay(40);
  #endif
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ disabled 
   
  #ifndef _DEBUG_ 
    delay (100);
  #endif  
}
