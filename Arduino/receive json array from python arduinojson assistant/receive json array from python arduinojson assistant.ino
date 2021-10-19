#include <ArduinoJson.h>
  
StaticJsonDocument<1008> doc;


void setup() {
  Serial.begin(9600);
}

void loop() {

  if(Serial.available()){
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    JsonArray index_of_point = doc["index_of_point"];
    int index_of_point_array[15];
    int i = 0;
    for(JsonVariant index : index_of_point){
      index_of_point_array[i] = index;
      i++;
    }

    JsonArray interpolation = doc["interpolation"];
    int interpolation_array[15];
    i = 0;
    for(JsonVariant interpol : interpolation){
      interpolation_array[i] = interpol;
      i++;
    }

    JsonArray velocity = doc["velocity"];
    int velocity_array[15];
    i = 0;
    for(JsonVariant vel : velocity){
      velocity_array[i] = vel;
      i++;
    }

    JsonArray acceleration = doc["acceleration"];
    int acceleration_array[15];
    i = 0;
    for(JsonVariant acc : acceleration){
      acceleration_array[i] = acc;
      i++;
    }

    JsonObject coordinates = doc["coordinates"];

    JsonArray coordinates_x = coordinates["x"];
    float x_array[15];
    i = 0;
    for(JsonVariant x_coord : coordinates_x){
      x_array[i] = x_coord;
      i++;
    }

    JsonArray coordinates_y = coordinates["y"];
    float y_array[15];
    i = 0;
    for(JsonVariant y_coord : coordinates_y){
      y_array[i] = y_coord;
      i++;
    }

    JsonArray coordinates_z = coordinates["z"];
    float z_array[15];
    i = 0;
    for(JsonVariant z_coord : coordinates_z){
      z_array[i] = z_coord;
      i++;
    }
    
    String message;

    for(int i = 0; i < 10; i++){ 
      message += " Index_of_point: "; message += index_of_point_array[i]; 
      message += " x:"; message += x_array[i]; 
      message += " y:"; message += y_array[i]; 
      message += " z:"; message += z_array[i];
      message += " Interpolation: "; message += interpolation_array[i];
      message += " Velocity: "; message += velocity_array[i]; 
      message += " Acceleration: "; message += acceleration_array[i];
    }
    Serial.println(x_array[14]);
    Serial.println(y_array[14]);
    Serial.println(z_array[14]);
    Serial.println(message);
    Serial.flush();
  }
}
