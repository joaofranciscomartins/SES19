#include <Wire.h>

#define OFF 0 // LED is off

#define BRIGHT 7   //Led is fully on
#define SURR 8     // Led is in surrounding mode
#define SAFE 9     // Led is in safe mode

#define YELLOW 4  // yellow led digital pin
#define RED1 3    // red led digital pin
#define RED2 5    // green led digital pin

#define BUTTON 6 //button digital pin

#define xCoordinate 10 // digital pin
#define yCoordinate 11 // digital pin

union coordinate {
  byte value;
  struct {
    byte x:4;
    byte y:4;
  };
};
const int POTENCIOMETER = A2; // potenciometer analog pin
const int LIGHT = A3;         // light sensor analog pin

const int RED1OK = A1;
const int RED2OK = A0;

const int OK = 10;        // check if RED1 Led is ok
const int NOTOK = 11;     // check if RED1 Led is ok

const int SURROUNDING = 128;
const int SAFETY = 64;

int adress = 0;
int x1 = 0;
int y1 = 0;

int x2 = 0;
int y2 = 0;

int identifier1 = 0;
int identifier2 = 0;

int lightValue = 0;         // value of light
int potenciometerValue = 0; // value of potenciometer

int interval = 10000;

// light calibration variables
int lightMin = 1023;
int lightMax = 0;

// potenciometer calibration variables
int potenciometerMin = 1023;
int potenciometerMax = 0;

unsigned long timeRed1ON = 0; // previous reading of the milliseconds
unsigned long timeRed2ON = 0; // previous reading of the milliseconds

int stateRed1 = OFF; // 
int stateRed2 = OFF; // 

int currPot = OFF; //Potenciometer current state
int prevPot = OFF; // Potenciometer previous state

int currStateLed1 = OK; //current state of the Red led 1
int prevStateLed1 = OK;

int currStateLed2 = OK; // current state of the Red led 2
int prevStateLed2  = OK;
int valRED1 = 0; //value of the state of the led
int valRED2 = 0; // value of the state of the led

int average = 0; //used to check if there are any errors

int faults = 0; //how many faults were received
int repairs = 0; // how many "is ok " where received

unsigned long timeRed1Surr = 0; //time led Red1 is at level = "surround"
unsigned long timeRed2Surr = 0; //time led Red2 is at level = "surround"

int pastcell1 = 0;
int pastcell2 = 0;

int delta = 0;
int numberN = 0;

unsigned long timestamp1 = 0;
unsigned long timestamp2 = 0;

unsigned long time_difference = 0;
int isButtonPressed() {
  return digitalRead(BUTTON) == HIGH;
}

void calibrateLight() {
 while (millis() < 10000) {
    lightValue = analogRead(LIGHT);

    // record the maximum sensor value
    if (lightValue > lightMax) {
      lightMax = lightValue;
    }

    // record the minimum sensor value
    if (lightValue < lightMin) {
      lightMin = lightValue;
    }
  }
}

void calibratePotenciometer() {
  while (millis() < 10000) {
    potenciometerValue = analogRead(POTENCIOMETER);
    
    // record the maximum sensor value
    if (potenciometerValue > potenciometerMax) {
      potenciometerMax = potenciometerValue;
    }

    // record the minimum sensor value
    if (potenciometerValue < potenciometerMin) {
      potenciometerMin = potenciometerValue;
    }
  }
}

void light() { // RED1 and RED2
  // reads the light value
  lightValue = analogRead(LIGHT);
  
  // map the value read by the light sensor in a valid range (0, 255)
  lightValue = map(lightValue, lightMin, lightMax, 0, 255);
  
  // in case the sensor value is outside the range seen during calibration
  lightValue = constrain(lightValue, 0, 255);
  
    // if it is night time
    if(lightValue < 30){
      if(stateRed1 != BRIGHT && stateRed1 != SURR){
        analogWrite(RED1, SAFETY);
        stateRed1 = SAFE;
      }
      if(stateRed2 != BRIGHT && stateRed2 != SURR){
        analogWrite(RED2, SAFETY);
        stateRed2 = SAFE;
      }
    }
    else{
      analogWrite(RED1, 0);
      stateRed1 = OFF;
      analogWrite(RED2, 0);
      stateRed2 = OFF;
    }
}

void potenciometer() { // RED1 LED
  // read the value from the sensor:
  potenciometerValue = analogRead(POTENCIOMETER);
  
  // map the value read by the light sensor in a valid range (200, 2000)
  potenciometerValue = map(potenciometerValue, potenciometerMin, potenciometerMax, 200, 2000);
  
  // in case the sensor value is outside the range seen during calibration
  potenciometerValue = constrain(potenciometerValue, 200, 2000);

  if (1100 <= (unsigned int) potenciometerValue) {
      currPot = BRIGHT;

      if( currPot != prevPot){
        analogWrite(RED1, 255);
        stateRed1 = BRIGHT;
        checkMovement(pastcell1, 1);

        // if led not fully on
        if(stateRed2 != BRIGHT){
           analogWrite(RED2,SURROUNDING); 
           stateRed2 = SURR;
        }

        timeRed1ON = millis();
        prevPot = BRIGHT;
        //send the I2C bus to all neighbours
          sendI2CBUS( int((x1 -1)/2 )*16 + y1,x1-1,y1,identifier1,1); 
          sendI2CBUS( int(x1/2)*16 + (y1-1),x1,y1-1, identifier1,1);
          sendI2CBUS( int((x1-1)/2)*16 + (y1-1),x1-1,y1-1, identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + y1,x1+1,y1,identifier1,1);
          //sendI2CBUS( int(x1/2)*16 + (y1+1), identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + (y1+1),x1+1,y1+1,identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + (y1-1),x1+1,y1-1,identifier1,1);
          sendI2CBUS( int((x1-1)/2)*16 + (y1+1),x1-1,y1+1, identifier1,1);
        
      }   
  }
 
  else{
      currPot = OFF;
      if( currPot != prevPot){
          analogWrite(RED1, 255);
          stateRed1 = BRIGHT;
          checkMovement(pastcell1, 1);
          if(stateRed2 != BRIGHT){
             analogWrite(RED2,SURROUNDING);
             stateRed2 = SURR;
          }
          timeRed1ON = millis();
          prevPot = OFF;

         //send the I2C bus to all neighbours
          sendI2CBUS( int((x1 -1)/2 )*16 + y1,x1-1,y1,identifier1,1); 
          sendI2CBUS( int(x1/2)*16 + (y1-1),x1,y1-1, identifier1,1);
          sendI2CBUS( int((x1-1)/2)*16 + (y1-1),x1-1,y1-1, identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + y1,x1+1,y1,identifier1,1);
          //sendI2CBUS( int(x1/2)*16 + (y1+1), identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + (y1+1),x1+1,y1+1,identifier1,1);
          sendI2CBUS( int((x1+1)/2)*16 + (y1-1),x1+1,y1-1,identifier1,1);
          sendI2CBUS( int((x1-1)/2)*16 + (y1+1),x1-1,y1+1, identifier1,1);
        }   
  }
}

void button() { // RED2 LED
  
  int buttonPressed = isButtonPressed();
  
  if (buttonPressed) {
    analogWrite(RED2, 255);
    stateRed2 = BRIGHT;
    checkMovement(pastcell2, 2);

    if(stateRed1 != BRIGHT){
         analogWrite(RED1,SURROUNDING); 
         stateRed1 = SURR;
      }
    
    timeRed2ON = millis();

     //send the I2C bus to all neighbours 
     //sendI2CBUS(( int((x2-1)/2)*16 + y2),identifier2,1); 
     sendI2CBUS(( int(x2/2)*16 + (y2-1)),x2,y2-1, identifier2,1);
     sendI2CBUS(( int((x2-1)/2)*16 + (y2-1)),x2-1,y2-1, identifier2,1);
     sendI2CBUS(( int((x2+1)/2)*16 + y2),x2+1,y2, identifier2,1);
     sendI2CBUS(( int(x2/2)*16 + (y2+1)),x2,y2+1, identifier2,1);
     sendI2CBUS(( int((x2+1)/2)*16 + (y2+1)),x2+1,y2+1,identifier2,1);
     sendI2CBUS(( int((x2+1)/2)*16 + (y2-1)),x2+1,y2-1,identifier2,1);
     sendI2CBUS(( int((x2-1)/2)*16 + (y2+1)),x2-1,y2+1, identifier2, 1);

  }
}
//see if there are failures in the leds
void checkLighting() { // RED1 and RED2
    average = 0;
    // does 5 measures and does the average because the signal is analog
    for(int i = 0; i < 5 ; i++){
     average += analogRead(RED1OK);
     delay(20);
    }
    
     valRED1 = average/5;

    if(stateRed1 == OFF && valRED1 == 0){
      currStateLed1 = OK;
      if(currStateLed1 != prevStateLed1){
        //send message on the I2C BUS everything is OK
        if(adress != 0){
          sendI2CBUS(0,identifier1,0,0,255);
        }
        else{
          lampRepaired(); //this is the controller 0
        }
        prevStateLed1 = currStateLed1;
      }
    }
     else if(stateRed1 != OFF && valRED1 == 0) {
       currStateLed1 = NOTOK;
        if(currStateLed1 != prevStateLed1){
          //send message on the I2C BUS lamp failed
          if(adress != 0){
          sendI2CBUS(0,identifier1,0,0,254);
          }
          else{
            lampFailure(); //this is the controller 0
          }
          prevStateLed1 = currStateLed1;
        }
     }
     else{
      currStateLed1 = OK;
       if(currStateLed1 != prevStateLed1){
          //send message on the I2C BUS everything is OK
          if(adress != 0){
             sendI2CBUS(0,identifier1,0,0,255);
          }
          else{
            lampRepaired(); //this is the controller 0
          }
          prevStateLed1 = currStateLed1;
      }
     }
   average = 0;
   
   for(int i = 0; i < 5 ; i++){
     average += analogRead(RED2OK);
     delay(20);
    }
    
     valRED2 = average/5;
    if(stateRed2 == OFF && valRED2 == 0){
      currStateLed2 = OK;
      if(currStateLed2 != prevStateLed2){
        //send message on the I2C BUS everything is OK
        if(adress != 0){
             sendI2CBUS(0,identifier2,0,0,255);
          }
        else{
            lampRepaired(); //this is the controller 0
          }
        prevStateLed2 = currStateLed2;
      }
    }
     else if(stateRed2 != OFF && valRED2 == 0) {
       currStateLed2 = NOTOK;
        if(currStateLed2 != prevStateLed2){
          //send message on the I2C BUS
          if(adress != 0){
             sendI2CBUS(0,identifier2,0,0,254);
          }
          else{
            lampFailure(); //this is the controller 0
          }
          prevStateLed2 = currStateLed2;
        }
     }
    else{
      currStateLed2 = OK;
       if(currStateLed2 != prevStateLed2){
          //send message on the I2C BUS everything is OK
          if(adress != 0){
             sendI2CBUS(0,identifier2,0,0,255);
          }
          else{
            lampRepaired(); //this is the controller 0
          }
          prevStateLed2 = currStateLed2;
      }
   } 
}
// if received a message that there is movement near by
void contiguousSurr(int xId, int yId){
  //int xId = int(identifier / 10);
  //int yId = identifier - (xId * 10); // could be done with % but this way is faster
  
  // we need to find out which of the cells is contiguous to the one that has movement
  if(((abs(xId - x1) < 2) && (abs(yId - y1) < 2 ))){
     if(stateRed1 != BRIGHT){
         analogWrite(RED1,SURROUNDING); //later will be an I2C BUS
         stateRed1 = SURR;
         timeRed1Surr = millis();
         
         pastcell1 = xId*10+yId;;
         timestamp1 = millis() - time_difference;
      }
   }

  
  if (((abs(xId - x2) < 2) && (abs(yId - y2) < 2 ))){
     if(stateRed2 != BRIGHT){
         analogWrite(RED2,SURROUNDING); 
         stateRed2 = SURR;
         timeRed2Surr = millis();
         
         pastcell2 = xId*10+yId;
         timestamp2 = millis() - time_difference;
      }
  }
}

// if received a message that there is movement coming on its way
void contiguousBright(int xId, int yId){
  //int xId = int(identifier / 10);
  //int yId = identifier - (xId * 10); // could be done with % but this way is faster

  // we need to find out which of the cells is contiguous to the one that has movement
  if(((abs(xId - x1) < 2) || (abs(yId - y1) < 2 ))){
      analogWrite(RED1, 255);
      stateRed1 = BRIGHT;
      timeRed1ON = millis();
   }

  if (((abs(xId - x2) < 2) && (abs(yId - y2) < 2 ))){
     analogWrite(RED2,255); 
     stateRed2 = BRIGHT;
     timeRed2ON = millis();
  }
}
// faulty lamp
void lampFailure(){
  faults +=1;
  digitalWrite(YELLOW, HIGH);
}
//lamp repaired
void lampRepaired(){
  repairs +=1;
  if (faults == repairs){
     digitalWrite(YELLOW, LOW);
  }
}

void checkMovement(int pastcell, int currcell){

  int xId = int(pastcell / 10);
  int yId = pastcell - (xId * 10);
  int deltaX = 0;
  int deltaY = 0;
  unsigned long speed = 0;

  if( currcell == 1 ){
    unsigned long deltaTime = (millis() - timestamp1) / 1000;
    deltaX = (x1 - xId) * 40;
    deltaY = (y1 - yId) * 40;
    int distance = int(sqrt(deltaX * deltaX + deltaY*deltaY));
    speed = (distance / deltaTime);
    if( speed > 5.2 ){
      sendI2CBUS(( int((x1+deltaX)/2)*16 + (y1+deltaY)),x1+deltaX,y1+deltaY, identifier1, 2);
    }
  }
  
  else{
        
    unsigned long deltaTime = (millis() - timestamp1) / 1000;
    deltaX = (x2 - xId) * 40;
    deltaY = (y2 - yId) * 40;
    
    unsigned long  distance = (sqrt(deltaX * deltaX + deltaY*deltaY));
    speed = (distance / deltaTime);

    if( speed > 5.2 ){
      sendI2CBUS(( int((x2+deltaX)/2)*16 + (y2+deltaY)),x2+deltaX,y2+deltaY, identifier2, 2);
    }
  }
}

void calculateTime(unsigned long timestamp){
  numberN +=1;
  delta += int((millis()+10) - timestamp);
}

void sendTime(){
 sendI2CBUS(( int((x1)/2)*16 + (y1+1)), x1, y1+1, identifier1,0);
 sendI2CBUS(( int((x1-1)/2)*16 + (y1)),x1-1,y1, identifier1,0);
 sendI2CBUS(( int((x1)/2)*16 + (y1-1)),x1,y1-1, identifier1,0);
 
 sendI2CBUS(( int((x2+1)/2)*16 + (y2)),x2+1,y2, identifier2,0);
 sendI2CBUS(( int((x2)/2)*16 + (y2+1)),x2,y2+1, identifier2,0);
 sendI2CBUS(( int((x2)/2)*16 + (y2-1)),x2,y2-1, identifier2,0);

}
void sendI2CBUS(int destination, int identifier,int xDest, int yDest, int event) {
  // begin master transmission to slave at the specifed address
  coordinate id = {0};
  
  if(identifier == identifier1){
    id.x = x1;
    id.y = y1;
  }
  else{
    id.x = x2;
    id.y = y2;
  }
  coordinate dest = {0};
  dest.x = xDest;
  dest.y = yDest;
  
  Wire.beginTransmission(destination);
  
  // writes a signal for slave initiate the respective procedure
  //Wire.write(destination);
  //Wire.write(identifier);
  //Wire.write(event);

  Wire.write(dest.value);
  Wire.write(id.value);
  Wire.write(lowByte(event));
  
  unsigned long timeStamp = millis();
  Wire.write(timeStamp);
  
  // end the master transmission
  Wire.endTransmission();
}

void i2cReader(int bytesRead) {
coordinate destination = {0};
coordinate identifier = {0};

  while (1 < Wire.available()) {
    destination.value = Wire.read();
    // read the first byte as an int (identifier)
    identifier.value = Wire.read();
    // read the second byte as an int (operation)
    int event = Wire.read();
    unsigned long timeStamp = Wire.read();
  //  Serial.println(destination);
  //  Serial.println(identifier);
  //  Serial.println(event);

    switch (event) {
      case 0:
        calculateTime(timeStamp);
        break;
      case 1:
        contiguousSurr(identifier.x,identifier.y); // see what cell is countiguous of the one that has movement and put it to surrounding state;
        break;
      case 2:
        contiguousBright(identifier.x,identifier.y); // see what cell is countiguous of the one that has movement and put it to bright state;
        break;
      case 254:
        lampFailure(); //receive the failure of a lamp, put yellow led on
        break;
      case 255:
        lampRepaired(); //receive the repair of a lamp, put yellow led off
        break;
      default:
        Serial.println("ERROR: Unknown Operation!");
    }
    
  }
}

void setup() {

  Serial.begin(9600);

  //SETUP THE COORDINATES BY RECEIVING INPUT
  pinMode(xCoordinate, INPUT);
  pinMode(yCoordinate, INPUT);

  x1 = digitalRead(xCoordinate);
  y1 = digitalRead(yCoordinate);
  Serial.println(x1);
  Serial.println(y1);
  x2 = x1+1;
  y2 = y1;

  //calculating the adress of this device in order to receive I2C messages
  adress = (int(x1/2)*16 + y1);
  Serial.println(adress);
  //the personal identifier of each cell
  identifier1 = x1*10 +y1;
  identifier2 = x2*10 +y2;

  //sendTime();
  
  time_difference = (delta / numberN);
  
  Wire.begin(adress); 
  Wire.onReceive(i2cReader);
  // define digital pins (button) as input
  pinMode(BUTTON, INPUT);

  // define digital pins as output
  pinMode(YELLOW, OUTPUT);
  pinMode(RED1, OUTPUT);
  pinMode(RED2, OUTPUT);
  
  // turn on all LEDs to signal calibration
  digitalWrite(YELLOW, HIGH);
  digitalWrite(RED1, HIGH);
  digitalWrite(RED2, HIGH);

  calibrateLight();
  calibratePotenciometer();

  // turn OFF all LEDs to signal calibration
  digitalWrite(YELLOW, LOW);
  digitalWrite(RED1, LOW);
  digitalWrite(RED2, LOW);
}

void loop() {
  checkLighting();
  //light();

  //if(stateRed1 != OFF){
      potenciometer();
  //}
//  if(stateRed2 != OFF){
      button();
  //}
  
//10 seconds until potentiometer is off
if ((millis() - timeRed1ON) >= interval &&  timeRed1ON != 0) {
     digitalWrite(RED1, LOW); //volta ao modo de seguranca
     stateRed1 = OFF;

     if(stateRed2 != BRIGHT  && timeRed2Surr == 0){
        digitalWrite(RED2,LOW); //later will be an I2C BUS
        stateRed2 = OFF;
     }
     timeRed1ON = 0;
  }

  //10 seconds until button is off
  if ((millis() - timeRed2ON) >= interval && timeRed2ON != 0) {
     digitalWrite(RED2, LOW); //volta ao modo de seguranca
     stateRed2 = OFF;

     if(stateRed1 != BRIGHT && timeRed1Surr ==0){
         digitalWrite(RED1,LOW); //later will be an I2C BUS
         stateRed1 = OFF;
      }
     timeRed2ON = 0;
  }
  
   if (stateRed1 != BRIGHT && (millis() - timeRed1Surr) >= interval && timeRed1Surr != 0) {
     timeRed1Surr= 0;
     digitalWrite(RED1, LOW); //volta ao modo de seguranca
     stateRed1 = OFF;    
  }
   if (stateRed2 != BRIGHT && (millis() - timeRed2Surr) >= interval && timeRed2Surr != 0) {
     timeRed2Surr= 0;
     digitalWrite(RED2, LOW); //volta ao modo de seguranca
     stateRed2 = OFF;
  }
}
