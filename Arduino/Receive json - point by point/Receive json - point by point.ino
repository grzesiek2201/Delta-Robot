#include <ArduinoJson.h>
#include <Wire.h>

StaticJsonDocument<256> doc_rx;

int interpolation_array[25];
int velocity_array[25];
int acceleration_array[25];
float x_array[25];
float y_array[25];
float z_array[25];
bool start = false;
bool home_cmd = false;
bool teach_in_cmd = false;
bool enable_cmd = false;
char mode = 0;

// class for handling encoders
class Encoder                         
{
  public:
    int adress;                         //encoder individual adress
    int magnet_status = 0;               //value of the status register (MD, ML, MH)


    int low_byte;                        //raw angle 7:0
    word high_byte;                      //raw angle 7:0 and 11:8
    int raw_angle;                       //final raw angle
    float deg_angle;                     //raw angle in degrees (360/4096 * [value between 0-4095])

    int quadrant_number, previous_quadrant_number;   //quadrant IDs
    float number_of_turns = 0;                      //number of turns
    float corrected_angle = 0;                     //tared angle - based on the startup value
    float start_angle = 0;                         //starting angle
    float total_angle = 0;                         //total absolute angular displacement
    float previous_total_angle = 0;                 //for the display printing


    //constructor
    Encoder (int Adress)
    {
      adress = Adress;
    }


    //function that makes Arduino reach out to the encoder and read the byte values
    void getRawData ()
    {
      //7:0 - bits
      Wire.beginTransmission(adress);           //connect to the sensor
      Wire.write(0x0D);                         //figure 21 - register map: Raw angle (7:0)
      Wire.endTransmission();                   //end transmission
      Wire.requestFrom(adress, 1);              //request from the sensor

      while(Wire.available() == 0);             //wait until it becomes available
      low_byte = Wire.read();                    //Reading the data after the request

      //11:8 - 4 bits
      Wire.beginTransmission(adress);
      Wire.write(0x0C);                         //figure 21 - register map: Raw angle (11:8)
      Wire.endTransmission();
      Wire.requestFrom(adress, 1);

      while(Wire.available() == 0);
      high_byte = Wire.read();
    }

    //processing byte values to a readable angle in deg
    void processData ()
    {
      //now there is 12 bit number distributed on two 8-bit registers
      //4 bits have to be shifted to its proper place as we want to build a 12-bit number
      high_byte = high_byte << 8;                 //shifting to left
      raw_angle = high_byte | low_byte;            //combine two 8bit registers onto one 16 bit register - after shiftng high_byte to the left
      // now there is 16bit register holding the value from range 0 to 4096 (12-bit max).

      deg_angle = raw_angle * 0.087890625;        // to calculate angle in deg- multiply raw_angle by constant
      // const is 360 (deg in turn) / 4096 (possible positions in turn as the sensor is 12bit)

    }

    //used in the setup() and it locks program until the magnet is not positioned properly
    void checkMagnet()
    {
      while ((magnet_status & 32) != 32)       //while the magnet is not adjusted to the proper distance - 32: MD = 1
      {
        magnet_status = 0;                       //reset reading
        Wire.beginTransmission(adress);         //connect to the sensor
        Wire.write(0x0B);                       //figure 21 - register map: Status: MD ML MH
        Wire.endTransmission();                 //end transmission
        Wire.requestFrom(adress, 1);            //request from the sensor
        while (Wire.available() == 0);          //wait until it becomes available
        magnet_status = Wire.read();             //Reading the data after the request
      }
    }

    void displayAngle()
    {
      Serial.println(deg_angle);
    }
};
Encoder encoder_1 (0x36);

void setup() {
  Serial.begin(9600);
}

void loop() {
  encoder_1.getRawData();
  encoder_1.displayAngle();
  delay(500);  
  if(Serial.available()){

    mode = Serial.read();
    Serial.println(mode);

    DeserializationError error = deserializeJson(doc_rx, Serial);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    
    if (mode == 48){ // 48 is 0 in ASCII
      start = doc_rx["start"];
      Serial.println(start);
      Serial.flush();
    }
    else if (mode == 49){ // 49 is 1 in ASCII
      int index_of_point = doc_rx["index_of_point"];
      interpolation_array[index_of_point] = doc_rx["interpolation"];
      velocity_array[index_of_point] = doc_rx["velocity"];
      acceleration_array[index_of_point] = doc_rx["acceleration"];
      JsonObject coordinates = doc_rx["coordinates"];
      x_array[index_of_point] = coordinates["x"];
      y_array[index_of_point] = coordinates["y"];
      z_array[index_of_point] = coordinates["z"];

      // just a random comment to delete
      Serial.println(index_of_point);
      Serial.println(interpolation_array[index_of_point]);
      Serial.println(velocity_array[index_of_point]);
      Serial.println(acceleration_array[index_of_point]);
      Serial.println(x_array[index_of_point]);
      Serial.println(y_array[index_of_point]);
      Serial.println(z_array[index_of_point]);
      Serial.flush();
    }
    else if (mode == 54){  //54 is 5 in ASCII, receive homing command and execute homing
      home_cmd = doc_rx["home_cmd"];
      // execute homing
      Serial.print("Home cmd: ");
      Serial.println(home_cmd);
    }

    else if (mode == 55){  //55 is 6 in ASCII, receive Teach-in command and send encoder positions
      teach_in_cmd = doc_rx["teach_in_cmd"];
      // execute teach in command - send encoders position
      encoder_1.displayAngle();
      //["Angle_1"] = read encoder 1;
      //...
      //serializeJson(doc_tx, output);
      //Serial.println(output);
      Serial.print("Teach_in cmd: ");
      Serial.println(teach_in_cmd);

      teach_in_cmd = false; // because it's a trigger button, the value has to be reset
    }

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
    }
  }
}

