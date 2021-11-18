// kompensacje do silników
// 1 - 165.0586
// 2 - 141.6797
// 3 - 140.3613

#include <ArduinoJson.h>
#include <Wire.h>

StaticJsonDocument<128> doc_rx;
StaticJsonDocument<64> doc_tx;
char output[64];

#define TIMEOUT 15000
#define BAUDRATE 9600

#define TCAADDR 0x70
#define address 0x36

// reading messages
const byte numChars = 128;
char receivedChars[numChars];
boolean newData = false;

// communication
int interpolation_array[25];
int velocity_array[25];
int acceleration_array[25];
float x_array[25];
float y_array[25];
float z_array[25];
bool start = false;
bool teach_in_cmd = false;
bool enable_cmd = false;
byte mode = 0;
bool receive_in_progress = false;

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

void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin();

  readRawAngle(1); readRawAngle(2); readRawAngle(3);
  startAngle[0] = 165.0586; 
  startAngle[1] = 141.6797; 
  startAngle[2] = 140.3613;
}

void loop() {
  // delay(2000);
  unsigned long time = millis();
  while (millis() - time < 2000){
    readRawAngle(1);
    readRawAngle(2);  
    readRawAngle(3);   
    correctAngle(1);
    correctAngle(2);
    correctAngle(3);
    checkQuadrant(1);
    checkQuadrant(2);
    checkQuadrant(3);
    sendEncodersData();
    delay(100);
  }
  // send a message saying 'I can receive'
  sendStartOfReceive();
  //count to 10 seconds, should be as long as it's needed to upload a program, no longer no shorter
  unsigned long receive_time = millis();  
  do {
      mode = readMode();  // read the message mode
      if (mode >= 48){   // if the message mode is an information message, process it
        receive_in_progress = true; // make sure all the message will be received
        delay(200);
        recvWithStartEndMarkers();  // read the message
        if (newData == true){ // if the message is read
          newData = false;  // reset the new data bool
          deserializeSerial();
          if (mode == 48){ // 48 is 0 in ASCII
            start = doc_rx["start"];
            Serial.println(start);
            Serial.flush();
          }
          else if (mode == 49){ // 49 is 1 in ASCII
            int index_of_point = doc_rx["n"];
            interpolation_array[index_of_point] = doc_rx["i"];
            velocity_array[index_of_point] = doc_rx["v"];
            acceleration_array[index_of_point] = doc_rx["a"];
            JsonArray coordinates = doc_rx["c"];
            x_array[index_of_point] = coordinates[0];
            y_array[index_of_point] = coordinates[1];
            z_array[index_of_point] = coordinates[2];

            Serial.println("OK");
            
            Serial.println(index_of_point);
            Serial.println(interpolation_array[index_of_point]);
            Serial.println(velocity_array[index_of_point]);
            Serial.println(acceleration_array[index_of_point]);
            Serial.println(x_array[index_of_point]);
            Serial.println(y_array[index_of_point]);
            Serial.println(z_array[index_of_point]);
            Serial.flush();
          }
          else if (mode == 54){  //54 is 6 in ASCII, receive homing command and execute homing
            // quadrantNumber[0] = 0;
            // quadrantNumber[1] = 0;
            // quadrantNumber[2] = 0;
            // previousquadrantNumber[0] = 0;
            // previousquadrantNumber[1] = 0;
            // previousquadrantNumber[2] = 0;
            numberofTurns[0] = 0;
            numberofTurns[1] = 0;
            numberofTurns[2] = 0;
            // execute homing
            Serial.print("Home cmd");
          }
          // else if (mode == 55){  //55 is 7 in ASCII, receive Teach-in command and send encoder positions
          //   teach_in_cmd = doc_rx["teach_in_cmd"];
          //   Serial.print("Teach_in cmd: ");
          //   Serial.println(teach_in_cmd);
          //   Serial.flush();
          //   teach_in_cmd = false; // because it's a trigger button, the value has to be reset
          // }
          else if (mode == 56){  //56 is 8 in ASCII, receive enable command and enable/disable motors accordingly
            enable_cmd = doc_rx["enable"];
            //if(enable){
            //  enable motors;
            //}
            //else{
            //  disable motors;
            //}

            Serial.print("Enable_cmd: ");
            Serial.println(enable_cmd);
            Serial.flush();
          }
        }
        memset(receivedChars, 0, sizeof(receivedChars));  // clear the variable where read bytes are being written into
      }
      else if (mode == '#'){  // '#' is 35 in ASCII, end of transmission
        receive_in_progress = false;
      }
  } while((millis() - receive_time < TIMEOUT) && receive_in_progress);
  sendEndOfReceive();
}


void sendStartOfReceive(){  // sends an information that robot controller is receiving data
  doc_tx["deg"][0] = -1111;
  doc_tx["deg"][1] = 0;
  doc_tx["deg"][2] = 0;
  serializeJson(doc_tx, output);
  Serial.println(output);
  Serial.flush();
}
void sendEndOfReceive(){  // sends an information that robot controller is no longer receiving data
  doc_tx["deg"][0] = -2222;
  doc_tx["deg"][1] = 0;
  doc_tx["deg"][2] = 0;
  serializeJson(doc_tx, output);
  Serial.println(output);
  Serial.flush();
}
void deserializeSerial(){
  DeserializationError error = deserializeJson(doc_rx, receivedChars);  // deserialize the message  
  if (error) {  // check for errors
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
  }
}
void recvWithStartEndMarkers(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
 
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
    }
}
void showNewData(){
  if (newData == true) {
    Serial.println(receivedChars);
    newData = false;
  }
}
char readMode(){
  recvWithStartEndMarkers(); // receive the message between < and > brackets
  if (newData == true) {
    Serial.print("Received mode: ");    
    Serial.println(receivedChars[0]);
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
  tcaselect(i);
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

  //while(Wire.available() == 0);
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
  // Serial.print("Corrected angle: ");
  // Serial.println(correctedAngle, 2); //print the corrected/tared angle  
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
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

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
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle[i] = (numberofTurns[i]*360) + correctedAngle[i]; //number of turns (+/-) plus the actual angle within the 0-360 range
  // Serial.print("Total angle: ");
  //Serial.println(-totalAngle/3, 2); //absolute position of the motor expressed in degree angles, 2 digits
  totalAngle[i] = int(totalAngle[i] * 100 / 3); // 3 is the gear ratio
  totalAngle[i] = float(totalAngle[i] / 100);
  doc_tx["deg"][i] = totalAngle[i];
  // doc_tx["deg"][i] = deg_angle[i];  
}
void sendEncodersData(){
  serializeJson(doc_tx, output);
  Serial.println(output);
  Serial.flush();
}