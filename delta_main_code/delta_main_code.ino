#include <ArduinoJson.h>
#include <Wire.h>
#include "inverse_kin.h"

StaticJsonDocument<64> doc_rx;
StaticJsonDocument<64> doc_tx;
char output[64];

#define TIMEOUT 5000
#define BAUDRATE 115200

#define ENC_COMPENSATE_1 109
#define ENC_COMPENSATE_2 156
#define ENC_COMPENSATE_3 155

#define TCAADDR 0x70
#define address 0x36

#define NO_POINTS 25

// reading messages
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;

// communication
uint8_t index_of_point;
uint8_t interpolation_array[NO_POINTS];
uint8_t velocity_array[NO_POINTS];
uint8_t acceleration_array[NO_POINTS];
float x_array[NO_POINTS];
float y_array[NO_POINTS];
float z_array[NO_POINTS];


bool start = false;
bool teach_in_cmd = false;
uint8_t enable_cmd = 0;
uint16_t program_length = 0;
byte mode = 0;
bool receive_in_progress = false;

// inverse kinematics
inverse_kin inverse_kin;
float position_array[3];
float previous_fi_array[3];
float fi_array[3];
short int steps_to_make_x[10];
short int steps_to_make_y[10];
short int steps_to_make_z[10];

// encoders
int low_byte;                        //raw angle 7:0
word high_byte;                      //raw angle 7:0 and 11:8
int raw_angle;                       //final raw angle
float deg_angle[3];                     //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber[3], previousquadrantNumber[3]; //quadrant IDs
float numberofTurns[3] = {0, 0, 0}; //number of turns
float correctedAngle[3] = {0, 0, 0}; //tared angle - based on the startup value
float startAngle[3] = {0, 0, 0}; //starting angle
float totalAngle[3] = {0, 0, 0}; //total absolute angular displacement
float previoustotalAngle[3] = {0, 0, 0}; //for the display printing


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// For RAMPS 1.4
#define X_DIR_PIN          55
#define X_STEP_PIN         54
#define X_ENABLE_PIN       38

#define Y_DIR_PIN          61
#define Y_STEP_PIN         60
#define Y_ENABLE_PIN       56

#define Z_DIR_PIN          48
#define Z_STEP_PIN         46
#define Z_ENABLE_PIN       62

#define X_STEP_HIGH             PORTF |=  0b00000001;
#define X_STEP_LOW              PORTF &= ~0b00000001;

#define Y_STEP_HIGH             PORTF |=  0b01000000;
#define Y_STEP_LOW              PORTF &= ~0b01000000;

#define Z_STEP_HIGH             PORTL |=  0b00001000;
#define Z_STEP_LOW              PORTL &= ~0b00001000;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);


struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned long minStepInterval; // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)
  volatile unsigned long estStepsToSpeed;  // estimated steps required to reach max speed
  volatile unsigned long estTimeForMove;   // estimated time (interrupt ticks) required to complete movement
  volatile unsigned long rampUpStepTime;
  volatile float speedScale;               // used to slow down this motor to make coordinated movement with other motors

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xStep() {
  X_STEP_HIGH
  X_STEP_LOW
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  Y_STEP_HIGH
  Y_STEP_LOW
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

void zStep() {
  Z_STEP_HIGH
  Z_STEP_LOW
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.rampUpStepTime = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 3

volatile stepperInfo steppers[NUM_STEPPERS];

// additional functions
struct pointFunction{
  bool func_type[3];
  unsigned int value[3];
  uint8_t pin_no[3];
};

void reset_point_functions(pointFunction* pt);
void reset_point_functions(pointFunction* pt){
  for(int i=0; i < NO_POINTS; i++){
    for(int j = 0; j < 3; j++){
      pt[i].func_type[j] = 0;
      pt[i].value[j] = 0;
      pt[i].pin_no[j] = 0;    
    } 
  }     
}

pointFunction point_functions[NO_POINTS];

void setup() {
  Serial.begin(BAUDRATE);
  Serial2.begin(BAUDRATE);
  Wire.begin();

  startAngle[0] = ENC_COMPENSATE_1; 
  startAngle[1] = ENC_COMPENSATE_2; 
  startAngle[2] = ENC_COMPENSATE_3;

  readRawAngle(1);
  readRawAngle(2);  
  readRawAngle(3);   
  correctAngle(1);
  correctAngle(2);
  correctAngle(3);
  checkQuadrant(1);
  checkQuadrant(2);
  checkQuadrant(3);
  fi_array[0] = totalAngle[0];
  fi_array[1] = totalAngle[1];
  fi_array[2] = totalAngle[2];

  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, HIGH);
  digitalWrite(Y_ENABLE_PIN, HIGH);
  digitalWrite(Z_ENABLE_PIN, HIGH);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = 500;
  steppers[0].minStepInterval = 30;

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = 500;
  steppers[1].minStepInterval = 30;

  steppers[2].dirFunc = zDir;
  steppers[2].stepFunc = zStep;
  steppers[2].acceleration = 500;
  steppers[2].minStepInterval = 30;

  while(Serial2.available()){ 
    char clear_input = Serial2.read();
    }
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
  si.speedScale = 1;

  float a = si.minStepInterval / (float)si.c0;
  a *= 0.676;

  float m = ((a*a - 1) / (-2 * a));
  float n = m * m;

  si.estStepsToSpeed = n;
}

volatile uint8_t remainingSteppersFlag = 0;

float getDurationOfAcceleration(volatile stepperInfo& s, unsigned int numSteps) {

  return s.c0 * sqrt(1.38 * numSteps + numSteps);
}

void prepareMovement(int whichMotor, long steps) {
  if (steps == 0) {return;}  
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  
  remainingSteppersFlag |= (1 << whichMotor);

  unsigned long stepsAbs = abs(steps);

  if ( (2 * si.estStepsToSpeed) < stepsAbs ) {
    // there will be a period of time at full speed
    unsigned long stepsAtFullSpeed = stepsAbs - 2 * si.estStepsToSpeed;
    float accelDecelTime = getDurationOfAcceleration(si, si.estStepsToSpeed);
    si.estTimeForMove = 2 * accelDecelTime + stepsAtFullSpeed * si.minStepInterval;
  }
  else {
    // will not reach full speed before needing to slow down again
    float accelDecelTime = getDurationOfAcceleration( si, stepsAbs / 2 );
    si.estTimeForMove = 2 * accelDecelTime;
  }
}

volatile uint8_t nextStepperFlag = 0;

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned long mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    TIMER1_INTERRUPTS_OFF
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
      s.rampUpStepTime += s.d;
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d * s.speedScale; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}


void runAndWait() {
  adjustSpeedScales();
  setNextInterruptInterval();
  TIMER1_INTERRUPTS_ON
  while ( remainingSteppersFlag )
  {
    // readEncoder(1);
    // readEncoder(2);
    // readEncoder(3);
    readEncoders();
    sendEncodersData();
    // delayMicroseconds(1000);
  };
  remainingSteppersFlag = 0;
  nextStepperFlag = 0;
}

void adjustSpeedScales() {
  float maxTime = 0;
  
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;
    if ( steppers[i].estTimeForMove > maxTime )
      maxTime = steppers[i].estTimeForMove;
  }

  if ( maxTime != 0 ) {
    for (int i = 0; i < NUM_STEPPERS; i++) {
      if ( ! ( (1 << i) & remainingSteppersFlag) )
        continue;
      steppers[i].speedScale = maxTime / steppers[i].estTimeForMove;
    }
  }
}

void loop() {
  int8_t point_number = 0;
  readEncoders();
  sendEncodersData();
  fi_array[0] = totalAngle[0];
  fi_array[1] = totalAngle[1];
  fi_array[2] = totalAngle[2];
  unsigned long time = millis();
  unsigned long receive_time = millis();  
  do {
      mode = readMode();  // read the message mode
      if (mode >= '0'){   // if the message mode is an information message, process it
        receive_in_progress = true; // make sure all the message will be received
        delay(10);
        recvWithStartEndMarkers();  // read the message
        if (newData == true){ // if the message is read
          newData = false;  // reset the new data bool
          deserializeSerial();
          switch(mode) {
            case '0':
              start = doc_rx["start"];
              break;

            case '1': // read program's points
              {                                 
              reset_point_functions(point_functions);   // reset additional functions              
              readPoint();
              Serial2.print("OK");
              Serial2.flush();  
              }
              program_length = index_of_point + 1;              
              break;
            case '2': // manual move
              readPoint();                           
              changeParameters(index_of_point);
              calculateSteps(index_of_point+1);
              prepareMovement(0, steps_to_make_x[index_of_point]);
              prepareMovement(1, steps_to_make_y[index_of_point]);
              prepareMovement(2, steps_to_make_z[index_of_point]);
              runAndWait();
              sendEncodersData();
              break;
            case '3': // Wait time         
              Serial2.print("OK");   
              point_number = doc_rx["pt_no"];
              point_functions[point_number].func_type[0] = 1;
              point_functions[point_number].value[0] = doc_rx["value"];
              break;   
            case '4': // Wait time
              Serial2.print("OK");
              point_number = doc_rx["pt_no"];
              point_functions[point_number].func_type[1] = 1;
              point_functions[point_number].value[1] = doc_rx["value"];
              point_functions[point_number].pin_no[1] = doc_rx["pin_no"];          
              break;            
            case '5': // Wait time
              Serial2.print("OK");
              point_number = doc_rx["pt_no"];            
              point_functions[point_number].func_type[2] = 1;
              point_functions[point_number].value[2] = doc_rx["value"];
              point_functions[point_number].pin_no[2] = doc_rx["pin_no"];
              break;            
            case '6':
              resetNumberOfTurns();
              break;
            case '8':
              enable_cmd = doc_rx["enable"];
              enableMotors();
              resetNumberOfTurns();
              readEncoders();
              fi_array[0] = totalAngle[0];
              fi_array[1] = totalAngle[1];
              fi_array[2] = totalAngle[2];
              break;
            
            default:
              break;                            
          }
        }
        memset(receivedChars, 0, sizeof(receivedChars));  // clear the variable where read bytes are being written into
      }
      else if (mode == '#'){  // '#' is 35 in ASCII, end of transmission
        receive_in_progress = false;
      }
  } while((millis() - receive_time < TIMEOUT) && receive_in_progress);
  
  if(start){
    calculateSteps(program_length);
    for(uint16_t i = 0; i < program_length; i++){
      changeParameters(i); 
      prepareMovement(0, steps_to_make_x[i]);
      prepareMovement(1, steps_to_make_y[i]);
      prepareMovement(2, steps_to_make_z[i]);
      runAndWait();
      sendEncodersData();
      if(point_functions[i].func_type[0]){   // "wait time" function  
        delay(point_functions[i].value[0]);
      }
      if(point_functions[i].func_type[1]){    // "wait input" function
        pinMode(point_functions[i].pin_no[2], INPUT);  
        while(digitalRead(point_functions[i].pin_no[1] != point_functions[i].value[1]));
      }
      if(point_functions[i].func_type[2]){    // "set output" function
        pinMode(point_functions[i].pin_no[2], OUTPUT);   
        digitalWrite(point_functions[i].pin_no[2], point_functions[i].value[2]);
      }
      delay(100);
    }
  }
}


void deserializeSerial(){
  DeserializationError error = deserializeJson(doc_rx, receivedChars);  // deserialize the message  
  if (error) {  // check for errors
    Serial2.print(F("deserializeJson() failed: "));
    Serial2.println(error.f_str());
  }
}
void recvWithStartEndMarkers(){
  static boolean recvInProgress = false;
  static byte norc = 0;
  char startSign = '<';
  char endSign = '>';
  char rc_byte;
 
  while (Serial2.available() > 0 && newData == false) {
    rc_byte = Serial2.read();

    if (recvInProgress == true) {
      if (rc_byte != endSign) {
        receivedChars[norc] = rc_byte;
        norc++;
        if (norc >= numChars) {
          norc = numChars - 1;
        }
      }
      else {
        receivedChars[norc] = '\0'; // terminate the string
        recvInProgress = false;
        norc = 0;
        newData = true;
      }
    }

    else if (rc_byte == startSign) {
      recvInProgress = true;
    }
    }
}
void showNewData(){
  if (newData == true) {
    Serial2.println(receivedChars);
    newData = false;
  }
}
char readMode(){
  recvWithStartEndMarkers(); // receive the message between < and > brackets
  if (newData == true) {
    // Serial2.print("Received mode: ");    
    // Serial2.println(receivedChars[0]);
    newData = false;
    return receivedChars[0];      
  }
  else return 0;
}
void tcaselect(uint8_t i){
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
void readRawAngle(short int i){
  i -= 1;
  Wire.beginTransmission(address);           //connect to the sensor
  Wire.write(0x0D);                         //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission();                   //end transmission
  Wire.requestFrom(address, 1);              //request from the sensor

  //while(Wire.available() == 0);            //wait until it becomes available
  low_byte = Wire.read();                    //Reading the data after the request

  //11:8 - 4 bits
  Wire.beginTransmission(address);
  Wire.write(0x0C);                         //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(address, 1);

  // while(Wire.available()== 0 );
  high_byte = Wire.read();

  //now there is 12 bit number distributed on two 8-bit registers
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  high_byte = high_byte << 8;                 //shifting to left
  raw_angle = high_byte | low_byte;            //combine two 8bit registers onto one 16 bit register - after shiftng high_byte to the left
  // now there is 16bit register holding the value from range 0 to 4096 (12-bit max).

  deg_angle[i] = raw_angle * 0.087890625;        // to calculate angle in deg- multiply raw_angle by constant
  // const is 360 (deg in turn) / 4096 (possible positions in turn as the sensor is 12bit)
  // deg_angle[i] = int(deg_angle * 100 / 3); // 3 is the gear ratio
  // deg_angle[i] = float(deg_angle / 100);
  // doc_tx["deg"][i] = - deg_angle;
}
void correctAngle(short int i){
  //recalculate angle
  i -= 1;
  correctedAngle[i] = deg_angle[i] - startAngle[i]; //this tares the position

  if(correctedAngle[i] < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle[i] = correctedAngle[i] + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  // Serial2.print("Corrected angle: ");
  // Serial2.println(correctedAngle, 2); //print the corrected/tared angle  
}
void checkQuadrant(short int i){
  /*
  //Quadrants:    //////////////// IS QUADRANT 2 AND 3 NECESSARY? ///////////////
  4  |  1
  ---|---
  3  |  2
  */

  i -= 1;  
  //Quadrant 1
  if(correctedAngle[i] >= 0 && correctedAngle[i] <=90)
  {
    quadrantNumber[i] = 1;
  }

  //Quadrant 2
  if(correctedAngle[i] > 90 && correctedAngle[i] <=180)
  {
    quadrantNumber[i] = 2;
  }

  //Quadrant 3
  if(correctedAngle[i] > 180 && correctedAngle[i] <=270)
  {
    quadrantNumber[i] = 3;
  }

  //Quadrant 4
  if(correctedAngle[i] > 270 && correctedAngle[i] <360)
  {
    quadrantNumber[i] = 4;
  }

  if(quadrantNumber[i] != previousquadrantNumber[i]) //if we changed quadrant
  {
    if(quadrantNumber[i] == 1 && previousquadrantNumber[i] == 4)
    {
      numberofTurns[i]++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber[i] == 4 && previousquadrantNumber[i] == 1)
    {
      numberofTurns[i]--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber[i] = quadrantNumber[i];  //update to the current quadrant
  
  }  
  //Serial2.print("Turns: ");
  //Serial2.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle[i] = (numberofTurns[i]*360) + correctedAngle[i]; //number of turns (+/-) plus the actual angle within the 0-360 range
  // Serial2.print("Total angle: ");
  //Serial2.println(-totalAngle/3, 2); //absolute position of the motor expressed in degree angles, 2 digits
  totalAngle[i] = int(totalAngle[i] * 100 / 3); // 3 is the gear ratio
  totalAngle[i] = float(totalAngle[i] / 100);
  doc_tx["deg"][i] = totalAngle[i];
  // doc_tx["deg"][i] = deg_angle[i];  
}
void readEncoders(){
    readEncoder(1);
    readEncoder(2);
    readEncoder(3);
}
void readEncoder(uint8_t i){
    tcaselect(i);    
    readRawAngle(i);
    correctAngle(i);
    checkQuadrant(i);
}
void sendEncodersData(){
  serializeJson(doc_tx, output);
  Serial2.println(output);
  Serial2.flush();
}
void calculateSteps(int index){
  for (int i = 0; i < index; i++){
    memcpy(previous_fi_array, fi_array, sizeof(fi_array));  //remember last angles              
    position_array[0] = x_array[i];
    position_array[1] = y_array[i];
    position_array[2] = z_array[i];
    if (inverse_kin.calculations(position_array, fi_array)){ // get new angles
      float delta_fi[3];
      for (int j = 0; j < 3; j++){
        delta_fi[j] = fi_array[j] - previous_fi_array[j];
      }
      steps_to_make_x[i] = 16 * 3 * delta_fi[0] / 1.8;
      steps_to_make_y[i] = 16 * 3 * delta_fi[1] / 1.8;
      steps_to_make_z[i] = 16 * 3 * delta_fi[2] / 1.8;
    }
    else{
      steps_to_make_x[i] = 0;
      steps_to_make_y[i] = 0;
      steps_to_make_z[i] = 0;
    }
    // Serial2.print("Steps_x: "); Serial2.println(steps_to_make_x[i]); 
    // Serial2.print("Steps_y: "); Serial2.println(steps_to_make_y[i]); 
    // Serial2.print("Steps_z: "); Serial2.println(steps_to_make_z[i]);   
    // Serial2.flush();  
  } 
}
void changeParameters(int index){
  steppers[0].acceleration = 100 / (acceleration_array[index] * 0.05);  
  steppers[1].acceleration = 100 / (acceleration_array[index] * 0.05);
  steppers[2].acceleration = 100 / (acceleration_array[index] * 0.05);

  steppers[0].minStepInterval = 100 / (velocity_array[index] * 0.3);
  steppers[1].minStepInterval = 100 / (velocity_array[index] * 0.3);
  steppers[2].minStepInterval = 100 / (velocity_array[index] * 0.3);
}
void resetNumberOfTurns(){
  numberofTurns[0] = 0;
  numberofTurns[1] = 0;
  numberofTurns[2] = 0;
}
void enableMotors(){
  digitalWrite(X_ENABLE_PIN, enable_cmd);
  digitalWrite(Y_ENABLE_PIN, enable_cmd);
  digitalWrite(Z_ENABLE_PIN, enable_cmd);  
}
void readPoint(){
  index_of_point = doc_rx["n"];
  interpolation_array[index_of_point] = doc_rx["i"];
  velocity_array[index_of_point] = doc_rx["v"];
  acceleration_array[index_of_point] = doc_rx["a"];
  JsonArray coordinates = doc_rx["c"];
  x_array[index_of_point] = coordinates[0];
  y_array[index_of_point] = coordinates[1];
  z_array[index_of_point] = coordinates[2];
}
