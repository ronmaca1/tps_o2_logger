//<debuuging defines>
#define   _DEBUG_ // for serial debugging
//#undef    _DEBUG_
//</debuging defines>

#define RXEN            LOW
#define RXDIS           HIGH
#define TXEN            HIGH
#define TXDIS           LOW

//<pin defines>
#define RS485RXEN       2 // LOW to enable
#define RS485TXEN       3 // HIGH to enable
#define DEBUG_HEARTBEAT 9 // tracking millis() accuracy
#define B1OXYGEN        A0
#define B2OXYGEN        A3
#define NOTUSED         A2
#define TPOS            A1
#define ERRORLED_RED    5 // pwm mode
#define ERRORLED_GREEN  6 // pwm mode
#define HEAT_1          7 // digital input
#define HEAT_2          8 // digital input
//</pin defines>
//<led defines>
#define OFF             0
#define RED_ON          32
#define GREEN_ON        127
//</led defines>
#include <Wire.h>
#include <stdio.h>
#include <PCF8583.h>
#include <SPI.h>
#include <SD.h>

PCF8583 myrtc (0xA0);
unsigned long startmillis = 0;
unsigned long currentmillis = 0;
unsigned long seconds = 0;
unsigned char loopcount = 0;

/*
long mymap(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}
*/
void setup() {
  // put your setup code here, to run once:
  //
  // first set adc reference to external to 
  analogReference(EXTERNAL);       // 2.5 Volt reference used
  // disable input buffers on ADC pins,
  // per datasheet page 43
  DIDR0 |= _BV(ADC3D) | _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D); //adc0 through 3

  // <set all unused pins to OUTPUT >  
  // digital pins
  int i;  
  const unsigned char unused_pins[1]{9};
  const size_t numunused = sizeof(unused_pins)/sizeof(unused_pins[0]);

    for(i=0;i<numunused;i++){
      digitalWrite(unused_pins[i],LOW);
      pinMode(unused_pins[i],OUTPUT);
      }
  // </set all unused pins to output and LOW>

  // <digital pins in use>
    
    
    pinMode (DEBUG_HEARTBEAT,OUTPUT);
    pinMode (RS485TXEN,OUTPUT);
    pinMode (RS485RXEN,OUTPUT);
    pinMode(HEAT_1,INPUT);
    pinMode(HEAT_2,INPUT);
  // </digital pins in use>
  
  //<pwm pins in use>
  analogWrite(ERRORLED_RED,OFF);
  analogWrite(ERRORLED_GREEN,OFF);
  //</pwm pins in use>

  //delay(20); // a short delay to let things stabilize
  #ifdef  _DEBUG_
  
  digitalWrite(RS485TXEN,TXEN);
  digitalWrite(RS485TXEN,RXDIS);
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
    while (1); // hang till power down and card inserted
    
  }
  #ifdef _DEBUG_
  Serial.println("card initialized.");
  #endif
  String  StartString = "";
  StartString += "Startup";
  digitalWrite(ERRORLED_RED,OFF); // all is well, get on with it
  digitalWrite(ERRORLED_GREEN,GREEN_ON);


  myrtc.get_time();
  char time[24];
  sprintf(time, "%02d/%02d/%02d %02d:%02d:%02d",
  myrtc.year, myrtc.month, myrtc.day, myrtc.hour, myrtc.minute, myrtc.second);
  Serial.println(time);
  StartString += time;
  digitalWrite(DEBUG_HEARTBEAT,LOW);
  
  startmillis = millis();
 
}


void loop() {
  // put your main code here, to run repeatedly:

int notused, tpos, b1oxygen, b2oxygen, temp;

      // Use of (byte) type casting and ascii math to achieve result.  
     if(Serial.available() > 0){
       myrtc.year= (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48)) + 2000;
       myrtc.month = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
       myrtc.day = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
       myrtc.hour  = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
       myrtc.minute = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
       myrtc.second = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48)); 

       if(Serial.read() == ';'){
         Serial.println("setting date");
         myrtc.set_time();
       }
    }


String dataString = "";
currentmillis = millis();
if(loopcount == 0){
  myrtc.get_time();
  char time[24];
  sprintf(time, "%02d/%02d/%02d %02d:%02d:%02d",
  myrtc.year, myrtc.month, myrtc.day, myrtc.hour, myrtc.minute, myrtc.second);
  
  dataString += String(time);
  dataString += ";\r\n";
  }
dataString += String(millis()-startmillis);
dataString += String(",");

 
  // <get us some heater info>
  dataString += String(digitalRead(HEAT_1));
  dataString += String(",");
  dataString += String(digitalRead(HEAT_2));
  dataString += String(",");
  temp = 0;
  // </get us some heater info>

  
  //<get us some o2 info (gain of 2(1))>
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp += analogRead(B1OXYGEN);
  temp = temp >> 2;  
  b1oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b1oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(1))>
  
  //<get us some o2 info (gain of 2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp += analogRead(B2OXYGEN);
  temp = temp >> 2;
  b2oxygen = map(temp,0,1023,0,1250);
  
  dataString += String(b2oxygen);
  dataString += String(",");
  //</get us some o2 info (gain of 2(2))>
  
  // <get us some throttle info (gain = 1/2(1))> 
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp += analogRead(TPOS);
  temp = temp >> 2;    
  tpos = map(temp,0,1023,0,5000);
  
  dataString += String(tpos);
  dataString += String(",");
  // </get us some throttle info>
  
  // <get us some unused channel (gain = 1/2(2))>
  temp = 0;
  // get 4 samples and then average them
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp += analogRead(NOTUSED);
  temp = temp >> 2;   
  notused = map(temp,0,1023,0,5000);
  
  dataString += String(notused); // last channel no comma appended
// </get us some unused channel (gain = 1/2(2))>

  // <SD card setup>
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);

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
    Serial.println("error opening datalog.csv");
    #endif
    analogWrite (ERRORLED_GREEN,GREEN_ON); // yellow if file error
    analogWrite (ERRORLED_RED,RED_ON);
  }
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ enabled
  /*
  #ifdef  _DEBUG_
    delay(40);
  #endif
  
  //should give us ~10 hertz sample rate
  // with _DEBUG_ disabled 
   
  #ifndef _DEBUG_ 
    delay (100);
  #endif  
  */

while (millis()-currentmillis < 100); // do every 100 millis aka 10 sample / sec.
 
if(loopcount <=600){
  loopcount++; 
  }else {
    loopcount = 0; //reset each minute 
  // see beginning of loop for the usage of this}
  }
digitalWrite(DEBUG_HEARTBEAT,!digitalRead(DEBUG_HEARTBEAT));
}
