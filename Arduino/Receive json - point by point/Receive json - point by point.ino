#include <ArduinoJson.h>

StaticJsonDocument<1008> doc;

int interpolation_array[15];
int velocity_array[15];
int acceleration_array[15];
float x_array[15];
float y_array[15];
float z_array[15];


void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    DeserializationError error = deserializeJson(doc, Serial);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    int index_of_point = doc["index_of_point"];
    interpolation_array[index_of_point] = doc["interpolation"];
    velocity_array[index_of_point] = doc["velocity"];
    acceleration_array[index_of_point] = doc["acceleration"];
    JsonObject coordinates = doc["coordinates"];
    x_array[index_of_point] = coordinates["x"];
    y_array[index_of_point] = coordinates["y"];
    z_array[index_of_point] = coordinates["z"];

    Serial.print(index_of_point);
    Serial.print(interpolation_array[index_of_point]);
    Serial.print(velocity_array[index_of_point]);
    Serial.println(acceleration_array[index_of_point]);
    Serial.println(x_array[index_of_point]);
    Serial.println(y_array[index_of_point]);
    Serial.println(z_array[index_of_point]);
    Serial.flush();
  }
}
