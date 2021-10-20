//    2021 DELTA robot software
//    Jakub Wrona & Grzegorz Radziwiłko
//    FOR NON COMMECIAL USE ONLY
//    Written in cooperation with Wroclaw University of Science and Technology under supervision of Dr. inż Krysztof Chrapek


//using namespace std;
#include <math.h>
#include <Wire.h>
#include <stdio.h>

#include "inverse_kin.h"
#include "multiplexer.h"


// For RAMPS 1.4  - PIN numbers for motors are defined by the hardware of RAMPS 1.4 board
// if necessairly, up to 5 motor slots are avaliable, for further pinout look into RAMPS1.4 datasheet

#define STEP_PIN_1         54
#define DIR_PIN_1          55
#define ENABLE_PIN_1       38

#define STEP_PIN_2         60
#define DIR_PIN_2          61
#define ENABLE_PIN_2       56

#define STEP_PIN_3         46
#define DIR_PIN_3          48
#define ENABLE_PIN_3       62

#define START_PROGRAM_PIN  11 // PIN NA KTÓRY DAMY SYGNAŁ STARTU PROGRAMU 

#define MAX_NUM_OF_POINTS  100

#define MICROSTEPPING      16
#define STEPS_PER_REV      200

#define LINEAR_APROX_RESOLUTION 1  //10 mm of distance between two points will result in generation of 10 intermediate points 

#define MEMORY_SAFETY_MARIGIN 10  //[%]  allocate x% more memmory than aproximated 



//define global variables 
const double STEP_ANGLE = 360 / STEPS_PER_REV;
int program_lenght=0; 
int program_converted_lenght=0;

int program_start=0;



///////////////////////  STRUCTURES AND CLASSES ///////////////////////

struct PositionData
{
  float a=NULL;                     //a data [deg]
  float b=NULL;                     //b data [deg]
  float c=NULL;                     //c data [deg]
  float x=NULL;                     //x data [mm] 
  float y=NULL;                     //y data [mm]  
  float z=NULL;                     //z data [mm] 
};

//Structure containing data about possible movement parameters of the motors 
struct MotionParam
{

  int min_interval=100;             //max speed - !!EXPERIMENTAL VALUE!!
  int max_interval=10000;           //min speed - !!EXPERIMENTAL VALUE!!

  int acc_slope_coeff=500;         //coefficient definig angle of the acceleration slope - !!EXPERIMENTAL VALUE!! take microstepping into account!

  float user_defined_speed_override=1;      //from GUI, [0-1]
  float user_defined_acc_override=1;        //from GUI, [0-1]

  float safety_override=1;                  //from safety measures like creating 'slow movement space' [0-1]

};

//Structure to contain movement point data 
struct Point
{

  byte index_of_point=NULL;         //point number in the program 
  byte interpolation=NULL;          //0=JOINT  1=LINEAR ... 2=CIRCULAR if implemented 
  byte speed=100;                   // [0-100%]
  byte acc=100;                     // [0-100%]
  float x=NULL;                     //x data [mm] 
  float y=NULL;                     //y data [mm]  
  float z=NULL;                     //z data [mm] 

};

//Structure to contain movement point data after convertion
struct PointConverted
{
  unsigned int point_of_origin=NULL; 
  byte interpolation=NULL;                //0=JOINT  1=LINEAR ... 2=CIRCULAR if implemented 
  unsigned int comp_reg_val_a=NULL;   //compare register value - defines how fast motor is stepping / rotating 
  unsigned int comp_reg_val_b=NULL;   
  unsigned int comp_reg_val_c=NULL;   
  int a=NULL;                             //number of steps motor1 
  int b=NULL;                             //number of steps motor2
  int c=NULL;                             //number of steps motor3 
  bool state_flag=false;                  //variable monitoring if efector already reached given point 

};


//Class for dealing with all the data and actions regarding stepper motors
class Motor
{
  public:

    int dirPin = 0;                   // Direction
    int stepPin = 0;                  // step
    int enablePin = 0;
    bool motor_state = false;         // variable for monitoring step pin state from previous cycle
    bool motion_done = false; 
    int steps_done = 0;               // variable for monitoring number of steps done by the motor in a given 'move' command
    int steps_required = 0;           // variable containing information about required amount of steps in a given 'move' command
    int timerNumber = 0;              // Identyfication number of the counter/timer that operate interrupt rutines for the PWM driving motor   ! must be 3, 4 or 5
  

    // constructor - setting pinout
    Motor(int dir_pin, int step_pin, int enable_pin, int timer_number)
    {
      dirPin=dir_pin;           // Direction
      stepPin=step_pin;         // step 
      enablePin=enable_pin;     // enable motors
      timerNumber= timer_number;
      pinMode(enablePin, OUTPUT);
      pinMode(dirPin, OUTPUT);
      pinMode(stepPin, OUTPUT);
    }
    
    // function to start moving in given direction
    void move (bool dir,unsigned int comp_register_value, int steps )
    {
      motion_done=false;
      steps_required = steps;
      if (dir)                                                         // set dir pin state accordingly to provided direction variable
      {
        digitalWrite(dirPin, HIGH);
      }
      else
      {
        digitalWrite(dirPin, LOW);
      }

      switch (timerNumber)
      {
        case 3:
          cli();                          //disable interrupts for modifiaction
          TCNT3 = 0;                      // set zero as counter value
          OCR3A = comp_register_value;    // set compare register value
          TIMSK3 |= (1 << OCIE3A);        // enable interrupts for this timer/counter
          sei();                          //enable interrupts for normal operation
          break;

        case 4:
          cli();
          TCNT4 = 0;
          TIMSK4 |= (1 << OCIE4A);
          OCR4A = comp_register_value;
          sei();
          break;

        case 5:
          cli();
          TCNT5 = 0;
          TIMSK5 |= (1 << OCIE5A);
          OCR5A = comp_register_value;
          sei();
          break;

        default:
          // error handling
          break;
      }
    }


    //function to stop moving
    void stopMove()
    {
      switch (timerNumber)
      {
        case 3:
          cli();                          //disable interrupts for modifiaction
          TIMSK3 = 0;                     //disable interrupts for this timer/counter
          sei();                          //enable interrupts for normal operation
          break;

        case 4:
          cli();
          TIMSK4 = 0;
          sei();
          break;

        case 5:
          cli();
          TIMSK5 = 0;
          sei();
          break;

        default:
          // error handling
          break;
      }
      steps_done = 0;                   //zero the values after making number of steps required by move command
      steps_required = 0;
      motion_done=true;
    }

    void enable ()
    {
      digitalWrite(enablePin, LOW);
    }

    void disable ()
    {
      digitalWrite(enablePin, HIGH);
    }

};


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

    int quadrant_number, previous_quadrant_number;  //quadrant IDs
    float number_of_turns = 0;                      //number of turns
    float corrected_angle = 0;                      //tared angle - based on the startup value
    float start_angle = 0;                          //starting angle
    float total_angle = 0;                          //total absolute angular displacement
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

      //while(Wire.available() == 0);            //wait until it becomes available
      low_byte = Wire.read();                    //Reading the data after the request

      //11:8 - 4 bits
      Wire.beginTransmission(adress);
      Wire.write(0x0C);                         //figure 21 - register map: Raw angle (11:8)
      Wire.endTransmission();
      Wire.requestFrom(adress, 1);

      //while(Wire.available() == 0);
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
        magnet_status = 0;                      //reset reading
        Wire.beginTransmission(adress);         //connect to the sensor
        Wire.write(0x0B);                       //figure 21 - register map: Status: MD ML MH
        Wire.endTransmission();                 //end transmission
        Wire.requestFrom(adress, 1);            //request from the sensor
        while (Wire.available() == 0);          //wait until it becomes available
        magnet_status = Wire.read();            //Reading the data after the request
      }
    }


    void displayAngle()
    {
      Serial.println(deg_angle);
    }

    float getAngleDeg ()
    {
      getRawData();
      processData();
      return deg_angle;
    }

};


///////////////////////  CREATING INSTANCES /////////////////////// 




// pin_dir  ;   pin_step ; enable_motor ; timerNumber
Motor motor_1(DIR_PIN_1, STEP_PIN_1, ENABLE_PIN_1, 3);      //create an instances of motor class, set the pins and the timer number
Motor motor_2(DIR_PIN_2, STEP_PIN_2, ENABLE_PIN_2, 4);
Motor motor_3(DIR_PIN_3, STEP_PIN_3, ENABLE_PIN_3, 5);

Encoder encoder_1 (0x70);     //create an instances of encoder class, set the adresses for I2C communication via I2C multiplexer module - set all adresses to multiplexer adress 
Encoder encoder_2 (0x70);     //changing the multiplexer channel when operating more than 1 encoder is required. 
Encoder encoder_3 (0x70);
 
Point * Program;
PointConverted * ProgramConverted;
PositionData PositionData;
Multiplexer Multiplexer(0x70);
inverse_kin inverse_kin;
MotionParam MotionParam;



///////////////////////  FUNCTIONS ///////////////////////


//fuction that moves all the motors synchronised
void moveAll(Motor &motor_1, Motor &motor_2, Motor &motor_3, int distance_1, int distance_2, int distance_3, bool dir_1, bool dir_2, bool dir_3, int interval)
{
  float longest_move = 0;
  float scalers[3] = {0};
  int distance_array[3] = {distance_1, distance_2, distance_3};
  //find the biggest distance
  for (int i=0; i<3; i++)
  {
    if (distance_array[i] > longest_move)
    {
      longest_move = distance_array[i];
    }
  }
  //calculate the scalers for each motor
  for(int i=0; i<3; i++){
    scalers[i] = distance_array[i] / longest_move;
    Serial.println(scalers[0]);
    Serial.println(scalers[1]);
    Serial.println(scalers[2]);
  }
  //scale the intervals and move all motors
  motor_1.move(dir_1, interval / scalers[0], distance_1);
  motor_2.move(dir_2, interval / scalers[1], distance_2);
  motor_3.move(dir_3, interval / scalers[2], distance_3);

}

// download robot program from the computer 
void getProgram()
{ 

  program_lenght=0;                //zero previous values 
  program_converted_lenght=0;
  free (Program);
  free (ProgramConverted);          //free memory allocated for previous program 
  // Set serial communication with Python program
  // get number of points in robot program.
  long int float_converter[4];      //shifting bits on float does not work :( 
  long int float_temp_accumulator[3];

  //komentarze po Polsku są robocze - do kasacji potem 

  program_lenght= 20;                 //z komunikacji przychodzi taka wartość, załóżmy że masz taką długość programu gdzie dla uproszczenia każdy punkt to 3x float 
                                    //wiadomo więc że przyjdzie 20 x 3 floaty czyli 20x3x4 bajty 


  byte myTransfer_packet_rxObj[240];      //dostajemy taką tablicę danych w bajtach 

  if (program_lenght>= MAX_NUM_OF_POINTS )
  {
    Serial.println("FATAL ERROR - TOO LONG PROGRAM");
  } 
  else 
  {
    Program = new Point [program_lenght];   //assign memory accordingly to number of points 
  }



  for (int i=0; i<program_lenght; i++)      //process data for every point 
  {

    for (int k=0; k<3; k++)                   // take floats in packets of 3 - that many floats is needed to fill one point 
    {

        for(int j=0; j<4; j++)                  // take bytes in packets of 4 - that many bytes is needed to combine into one float point of data 
        {
          float_converter[j]= (long int) myTransfer_packet_rxObj[12*i+4*k+j];  //from table of bytes, cast data onto long int variable. 
          //At one 'i' iteration, 12 data points will be consumed; respectively - at one 'k' iteration, 4 data points
        }
        float_converter[0] =  float_converter[0] << 24;      // most significant byte here  
        float_converter[1] =  float_converter[1] << 16;
        float_converter[2] =  float_converter[2] << 8;
        // float_converter[3] contains least significant byte, no shift needed 
        float_temp_accumulator[k] = float_converter[0] | float_converter[1] | float_converter[2] | float_converter[3];  //combine 4 bytes into one long int 
    }
 
    // check validity of data - interpolation must be 0,1 or 2, speed 0-100, acc 0-100 
    // if any data is invalid - signal an error and abort robot program download 
    Program[i].index_of_point=i;
    // Program[i].interpolation=NULL;
    // Program[i].speed=NULL;
    // Program[i].acc=NULL;
    Program[i].x= (float) float_temp_accumulator[0];
    Program[i].y= (float) float_temp_accumulator[1];
    Program[i].z= (float) float_temp_accumulator[2];  
  
  }

}

//take downloaded robot program and convert it so its ready to be run 
void decodeProgram() 
{
  int aprox_conv_prog_len=0;

  aprox_conv_prog_len = calculateApproxConvertedProgLenght();                             //calculate approximated converted program lenght 

  ProgramConverted = new PointConverted [aprox_conv_prog_len*(1+(MEMORY_SAFETY_MARIGIN/100))];        //allocate memory on converted program pointer, program lenght approxiated 


  for (int i=0; i<program_lenght; i++)                        // run functions (interpolation + kinematics + data transfer) for every user-defined  point in program 
  {
    if (Program[i].interpolation==0)              // interpolation 0 - joint 
    {
      jointInterpolation(i);
    }
    else if (Program[i].interpolation==1)         // interpolation 1 - linear
    {
      linearInterpolation(i);
    }
  }
}

void jointInterpolation(int p_index_number)        
{ 
  float temp_coordinates[3];
  float motor_angles[3];
  float angular_distance[3];
  int step_distance[3];
  int i = p_index_number;

  float general_speed_override =MotionParam.user_defined_speed_override * MotionParam.safety_override * Program[p_index_number].speed /100;    //calculate general speed override [0-1] 
  float general_acc_override =MotionParam.user_defined_acc_override * MotionParam.safety_override * Program[p_index_number].acc /100;          //calculate general acceleration override [0-1]
  if (general_speed_override>1) {general_speed_override=1;}
  if (general_acc_override>1)   {general_speed_override=1;}                                 //check if values are within logical limits [0-1]

  temp_coordinates[0]=Program[i].x;
  temp_coordinates[1]=Program[i].y;
  temp_coordinates[2]=Program[i].z;
  inverse_kin.calculations (temp_coordinates, (&motor_angles)[3]);                  // do inverese kinematics and get the angle data from motor_angles array 
  calculateAngularDistance ((&angular_distance)[3],motor_angles);
  for (int k=0; k<3; k++)                                                             //calculate distance from start of the move to a given intermediate point in steps 
  {
    step_distance[i]= (int) angular_distance[i] * STEPS_PER_REV / 360 * MICROSTEPPING; 
  }
  assignStepDistance (step_distance);
  ProgramConverted[program_converted_lenght].interpolation = Program[p_index_number].interpolation;
  ProgramConverted[program_converted_lenght].point_of_origin = p_index_number;


  program_converted_lenght++;                   //add 1 to converted program lenght - in joint interpolation there is going to be only one point added (with acceleration ramp) 
}

void linearInterpolation(int p_index_number)
{
  int i = p_index_number;
  int estimated_num_of_points=0;
  int already_added_points=0;
  float distance=0;
  float vector_array[3];
  float temp_coordinates[3];
  float motor_angles[3];
  float angular_distance[3];
  int step_distance[3];
  
  
  distance= calculateDistanceLine(i);                                     //get the linear distance to target 
  estimated_num_of_points = (int) distance * LINEAR_APROX_RESOLUTION;     //calculate how many intermediate points should be created to achevie linear movement with given resolution 
  returnDirectionVectors (i, estimated_num_of_points, (&vector_array)[3]);    //basing on number of intermediate points, calculate lenght of unit vectors (between intermediate points)

  getPos();
  
  for (int j=0; j<estimated_num_of_points-1; j++)                         //generate all but last intermediate points 
  {
    ProgramConverted[j].interpolation = Program[p_index_number].interpolation ;    //transfer interpolation data 
    ProgramConverted[j].point_of_origin = p_index_number;

    if (p_index_number==0)          // if its first point, calculate from actual effector position 
    {
      temp_coordinates[0] = PositionData.x + j * vector_array[0];                       //coordinates of point j =  actual position + j* unit vector of translation on each axis 
      temp_coordinates[1] = PositionData.y + j * vector_array[1];
      temp_coordinates[2] = PositionData.z + j * vector_array[2];  
      inverse_kin.calculations (temp_coordinates, (&motor_angles)[3]);                  // do inverese kinematics and get the angle data from motor_angles array 
      calculateAngularDistance ((&angular_distance)[3],motor_angles);
    } 
    else
    {
      temp_coordinates[0] = Program[p_index_number-1].x + j * vector_array[0];
      temp_coordinates[1] = Program[p_index_number-1].y + j * vector_array[1];
      temp_coordinates[2] = Program[p_index_number-1].z + j * vector_array[2];          // create xyz position data for given intermediate point basing on 'one point before' position  
      inverse_kin.calculations (temp_coordinates, (&motor_angles)[3]);                  // do inverese kinematics and get the angle data from motor_angles array 
      calculateAngularDistance ((&angular_distance)[3],motor_angles);
    }

    for (int k=0; k<3; k++)                                                             //calculate distance from start of the move to a given intermediate point in steps 
    {
      step_distance[i]= (int) angular_distance[i] * STEPS_PER_REV / 360 * MICROSTEPPING; 
    }

    assignStepDistance (step_distance);
    subtractPreviusPointsSteps (already_added_points);
    program_converted_lenght++;
  }

  // for the last point do different procedure- just assign data from oryginal target point 
  ProgramConverted[program_converted_lenght].interpolation = Program[p_index_number].interpolation ;    //transfer interpolation data 
  ProgramConverted[program_converted_lenght].point_of_origin = p_index_number;
  temp_coordinates[0] = Program[p_index_number].x ;
  temp_coordinates[1] = Program[p_index_number].y ;
  temp_coordinates[2] = Program[p_index_number].z ;
  inverse_kin.calculations (temp_coordinates, (&motor_angles)[3]); 
  calculateAngularDistance ((&angular_distance)[3],motor_angles); 
  for (int k=0; k<3; k++)                                                             //calculate distance from start of the move to a given intermediate point in steps 
  {
    step_distance[i]= (int) angular_distance[i] * STEPS_PER_REV / 360 * MICROSTEPPING; 
  }
  assignStepDistance (step_distance);
  subtractPreviusPointsSteps (already_added_points);
  program_converted_lenght++;
  calculateMacroRamp(already_added_points,p_index_number );                                                //after all step lenght are calculated, calculate and assignt compare registers values - defining wait time between steps 
}


void calculateMacroRamp (int num_of_intermediate_points_added, int p_index_number)
{

  float general_speed_override =MotionParam.user_defined_speed_override * MotionParam.safety_override * Program[p_index_number].speed /100;    //calculate general speed override [0-1] 
  float general_acc_override =MotionParam.user_defined_acc_override * MotionParam.safety_override * Program[p_index_number].acc /100;        //calculate general acceleration override [0-1]
  if (general_speed_override>1) {general_speed_override=1;}
  if (general_acc_override>1)   {general_speed_override=1;}                               //check if values are within logical limits [0-1]
  
  
  int ramp_lenght=0;                                                                      // lenght of acceleration and deacceleration [steps]
  int min_comp_reg_value = (int) MotionParam.min_interval / general_speed_override;       //divide min interval by speed override (0-1) to increase the interval, that is to decrease speed
  int max_comp_reg_value = (int) MotionParam.max_interval / general_speed_override;       
  ramp_lenght= MotionParam.acc_slope_coeff;                                               // define in how many steps, motor is supposed to accelerate to full speed
  int combined_points_steps[3];
  unsigned int previous_reg_value = max_comp_reg_value;
  int this_move_start= program_converted_lenght-num_of_intermediate_points_added;
  int missing_lenght=0;
  int short_ramp=0;

  for (int i=0; i<3; i++)                                                                 //calculate combined steps sum of all the points added in one linear interpolation command (for every axis)
  {
    combined_points_steps[i] = calculateCombinedStepsNum (num_of_intermediate_points_added,i);
  }

  ////////////////////////////  MOTOR_1  ////////////////////////////////

  if (2* ramp_lenght > combined_points_steps[0])                                        //FOR MOTOR 1 check if acceleration lenght +  deaccelarion lenght is greater than whole move lenght 
  {                                                                                     // in this case motor won't be able to reach full speed and there will be deacceleration right after accel 
    missing_lenght = 2* ramp_lenght - combined_points_steps[0]; 
    short_ramp = ramp_lenght - missing_lenght/2 ; 

    ProgramConverted[this_move_start].comp_reg_val_a = max_comp_reg_value;              // for first point set max comp_reg_value     
    for (int k=1; k<short_ramp; k++)                                                    // short acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_a = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value= ProgramConverted[this_move_start+k].comp_reg_val_a;
     
    }
    for (int k=0; k<short_ramp; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght + ramp_lenght + k].comp_reg_val_a = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_a;
    }

  } 
  else                // Here motor will reach full speed and move will look like this  - accel -> constant speed -> deaccel
  {                                                                                     
    ProgramConverted[this_move_start].comp_reg_val_a = max_comp_reg_value;    // for first point set max comp_reg_value 
      
    for (int k=1; k<ramp_lenght; k++)                                                       //acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_a = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value = ProgramConverted[this_move_start+k].comp_reg_val_a;
     
    }

    for (int k=ramp_lenght; k<num_of_intermediate_points_added-ramp_lenght; k++)            // constant speed 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_a = min_comp_reg_value;
    } 

    for (int k=0; k<ramp_lenght; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_a = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_a;
    }


  }


  ////////////////////////////  MOTOR_2  ////////////////////////////////

  if (2* ramp_lenght > combined_points_steps[1])                                        //FOR MOTOR 2 check if acceleration lenght +  deaccelarion lenght is greater than whole move lenght 
  {                                                                                     // in this case motor won't be able to reach full speed and there will be deacceleration right after accel 
    missing_lenght = 2* ramp_lenght - combined_points_steps[1]; 
    short_ramp = ramp_lenght - missing_lenght/2 ; 

    ProgramConverted[this_move_start].comp_reg_val_b = max_comp_reg_value;              // for first point set max comp_reg_value 
    for (int k=1; k<short_ramp; k++)                                                    // short acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_b = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value= ProgramConverted[this_move_start+k].comp_reg_val_b;
     
    }
    for (int k=0; k<short_ramp; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght + ramp_lenght + k].comp_reg_val_b = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_b;
    }

  } 
  else                // Here motor will reach full speed and move will look like this  - accel -> constant speed -> deaccel
  {                                                                                     
    ProgramConverted[this_move_start].comp_reg_val_b = max_comp_reg_value;    // for first point set max comp_reg_value 
      
    for (int k=1; k<ramp_lenght; k++)                                                       //acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_b = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value = ProgramConverted[this_move_start+k].comp_reg_val_b;
     
    }

    for (int k=ramp_lenght; k<num_of_intermediate_points_added-ramp_lenght; k++)            // constant speed 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_b = min_comp_reg_value;
    } 

    for (int k=0; k<ramp_lenght; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_b = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_b;
    }


  }


  ////////////////////////////  MOTOR_3  ////////////////////////////////

  if (2* ramp_lenght > combined_points_steps[2])                                        //FOR MOTOR 3 check if acceleration lenght +  deaccelarion lenght is greater than whole move lenght 
  {                                                                                     // in this case motor won't be able to reach full speed and there will be deacceleration right after accel 
    missing_lenght = 2* ramp_lenght - combined_points_steps[2]; 
    short_ramp = ramp_lenght - missing_lenght/2 ; 

    ProgramConverted[this_move_start].comp_reg_val_c = max_comp_reg_value;              // for first point set max comp_reg_value 
    for (int k=1; k<short_ramp; k++)                                                    // short acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_c = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value= ProgramConverted[this_move_start+k].comp_reg_val_c;
     
    }
    for (int k=0; k<short_ramp; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght + ramp_lenght + k].comp_reg_val_c = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_c;
    }

  } 
  else                // Here motor will reach full speed and move will look like this  - accel -> constant speed -> deaccel
  {                                                                                     
    ProgramConverted[this_move_start].comp_reg_val_c = max_comp_reg_value;    // for first point set max comp_reg_value 
      
    for (int k=1; k<ramp_lenght; k++)                                                       //acceleration ramp                 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_c = calculateRegisterValue(k,previous_reg_value);
      previous_reg_value = ProgramConverted[this_move_start+k].comp_reg_val_c;
     
    }

    for (int k=ramp_lenght; k<num_of_intermediate_points_added-ramp_lenght; k++)            // constant speed 
    {
      ProgramConverted[this_move_start+k].comp_reg_val_c = min_comp_reg_value;
    } 

    for (int k=0; k<ramp_lenght; k++)                                                       //deacceleration ramp                 
    {
      ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_c = calculateRegisterValueDown(k,previous_reg_value);
      previous_reg_value = ProgramConverted[program_converted_lenght - ramp_lenght + k].comp_reg_val_c;
    }


  }


  ////////////////////// ALL MOTORS AGAIN ///////////////////

  synchronizeMotorMovement(combined_points_steps, num_of_intermediate_points_added);


}



void synchronizeMotorMovement(int combined_motor_steps[3], int num_of_intermediate_points_added)
{
  int longest_move_steps = 0;
  float scalers[3] = {0};
  for (int i=0; i<3; i++)
  {
    if (combined_motor_steps[i] > longest_move_steps)
    {
      longest_move_steps = combined_motor_steps[i];
    }
  }

  for(int i=0; i<3; i++)
  {
    scalers[i] = combined_motor_steps[i] / longest_move_steps;
  }

  for (int k=0; k<num_of_intermediate_points_added; k++)                                                                
  {
    ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_a = ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_a / scalers[0] ; 
    ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_b = ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_b / scalers[1] ; 
    ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_c = ProgramConverted[program_converted_lenght - num_of_intermediate_points_added + k].comp_reg_val_c / scalers[2] ;
     
  }
}


unsigned int calculateRegisterValueDown (int n, unsigned int previous_reg_val)
{
  unsigned int reg_val=0;
  reg_val = previous_reg_val *   (4 * n +1 ) / ((4*n+1) - 2); 
  return reg_val=0;
}



unsigned int calculateRegisterValue (int n, unsigned int previous_reg_val)
{
  unsigned int reg_val=0;
  reg_val = previous_reg_val -  2 * previous_reg_val / (4 * n + 1);
  return reg_val=0;
}


int  calculateCombinedStepsNum (int num_of_intermediate_points_added, int axis)
{
  int sum[3];

  for (int i=0; i<num_of_intermediate_points_added; i++)                                  // for all added points, set the minimal interval (max speed)
  {
   sum[0]= sum[0] + ProgramConverted[program_converted_lenght-i].a;
   sum[1]= sum[1] + ProgramConverted[program_converted_lenght-i].b;
   sum[2]= sum[2] + ProgramConverted[program_converted_lenght-i].c;
  }

  return sum[axis];
}



void subtractPreviusPointsSteps (int already_added_points )
{
  for (int h=0; h<already_added_points; h++)              // from the step distance of the point, substract step distance of already added intermediate points - 
    {                                                       //only step distance from previously added position to most recent position remains 
      ProgramConverted[program_converted_lenght].a = ProgramConverted[program_converted_lenght].a - ProgramConverted[program_converted_lenght-h].a;
      ProgramConverted[program_converted_lenght].b = ProgramConverted[program_converted_lenght].b - ProgramConverted[program_converted_lenght-h].b;
      ProgramConverted[program_converted_lenght].c = ProgramConverted[program_converted_lenght].c - ProgramConverted[program_converted_lenght-h].c;
    }
}

void assignStepDistance (int step_distance[3])
{
  ProgramConverted[program_converted_lenght].a = step_distance[0];
  ProgramConverted[program_converted_lenght].b = step_distance[1];
  ProgramConverted[program_converted_lenght].c = step_distance[2];
}

void calculateAngularDistance (float (&angular_distance)[3], float motor_angles[3])
{
  angular_distance[0] = motor_angles[0] - PositionData.a;
  angular_distance[1] = motor_angles[1] - PositionData.b;
  angular_distance[2] = motor_angles[2] - PositionData.c;
}

void returnDirectionVectors (int i, int num_of_intermediate_points, float (&vector_array)[3])
{
  float diff_x=0;
  float diff_y=0;
  float diff_z=0;
  if (i==0)
      {
        getPos();
        diff_x = Program[i].x - PositionData.x;
        diff_y = Program[i].y - PositionData.y;
        diff_z = Program[i].z - PositionData.z;
      }else 
      {
        diff_x = Program[i].x - Program[i-1].x;
        diff_y = Program[i].y - Program[i-1].y;
        diff_z = Program[i].z - Program[i-1].z;
      }
     vector_array[0]= diff_x / num_of_intermediate_points;
     vector_array[1]= diff_y / num_of_intermediate_points;
     vector_array[2]= diff_z / num_of_intermediate_points;
}

float calculateDistanceLine (int i)
{
  float distance=0;
  float diff_x=0;
  float diff_y=0;
  float diff_z=0;
  if (i==0)
      {
        getPos();
        // here do forward kinematics of actual effector position
        diff_x = Program[i].x - PositionData.x;
        diff_y = Program[i].y - PositionData.y;
        diff_z = Program[i].z - PositionData.z;
      }else 
      {
        diff_x = Program[i].x - Program[i-1].x;
        diff_y = Program[i].y - Program[i-1].y;
        diff_z = Program[i].z - Program[i-1].z;
      }
      distance= sqrt ( sq(diff_x) + sq(diff_y)  + sq(diff_z) );
  return distance;
}

int calculateApproxConvertedProgLenght()
{
  int sum=0;
  float distance=0;
  int num_of_points=0;
  int i=0;

  for (int i=0; i<program_lenght; i++)
  {
    if (Program[i].interpolation==0)              // interpolation 0 - joint 
    {
      sum++;
    }
    else if (Program[i].interpolation==1)         // interpolation 1 - linear
    {
      distance= calculateDistanceLine(i);
      num_of_points = (int) distance * LINEAR_APROX_RESOLUTION  + 1;   
      sum = sum + num_of_points; 
    }
  }
  return sum;
}

void getPos()
{
  Multiplexer.selectChannel(0);                 // set channel on multiplexer, aiming at given encoder channel 
  PositionData.a = encoder_1.getAngleDeg();     // get angle data from given encoder 
  Multiplexer.selectChannel(1);
  PositionData.b = encoder_2.getAngleDeg();
  Multiplexer.selectChannel(2);
  PositionData.c = encoder_3.getAngleDeg();
  // here do forward kinematics of actual effector position

}

// execute decoded program 
void runProgram()
{

  for (int i=0; i<program_converted_lenght; i++)
  {
    if (i==0 || ProgramConverted[i-1].state_flag==true)           //go to next point if 1) its a first point  or 2) previous point was reached 
    {
      
      if (ProgramConverted[i].interpolation==0)                   // interpolation 0 - joint 
      {
      moveJoint(i);
      }
      else if (Program[i].interpolation==1)                       // interpolation 1 - linear
      {
      moveLinear(i);
      }
    }
  }
}

//Move effector to position 
void moveLinear(int converted_point_index )
{
  int i = converted_point_index;
  bool dir[3];  

  dir[0]=checkDir(ProgramConverted[i].a);                          //check the direction - check if steps>0 or steps<0
  dir[1]=checkDir(ProgramConverted[i].b);
  dir[2]=checkDir(ProgramConverted[i].c); 

  motor_1.move(dir[0],ProgramConverted[i].comp_reg_val_a, ProgramConverted[i].a);         //set the movement of the motors
  motor_2.move(dir[1],ProgramConverted[i].comp_reg_val_b, ProgramConverted[i].b);
  motor_3.move(dir[2],ProgramConverted[i].comp_reg_val_c, ProgramConverted[i].c);

  checkIfMoveDone();                                               //wait untill movement is completed an then proceed with the program 
  ProgramConverted[i].state_flag = true;                           // when movement is completed - mark the point as reached 
}



//Move effector to position 
void moveJoint(int converted_point_index )
{
  int i = converted_point_index;
  bool dir[3];  

  dir[0]=checkDir(ProgramConverted[i].a);                          //check the direction - check if steps>0 or steps<0
  dir[1]=checkDir(ProgramConverted[i].b);
  dir[2]=checkDir(ProgramConverted[i].c); 

  motor_1.move(dir[0],ProgramConverted[i].comp_reg_val_a, ProgramConverted[i].a);         //set the movement of the motors
  motor_2.move(dir[1],ProgramConverted[i].comp_reg_val_b, ProgramConverted[i].b);
  motor_3.move(dir[2],ProgramConverted[i].comp_reg_val_c, ProgramConverted[i].c);

  checkIfMoveDone();                                               //wait untill movement is completed an then proceed with the program 
  ProgramConverted[i].state_flag = true;                           // when movement is completed - mark the point as reached 
}


void checkIfMoveDone()
{
  while (motor_1.motion_done && motor_2.motion_done && motor_3.motion_done)
  {
    //wait here untill all the motors complete their movement 
  }
}

bool checkDir(int steps)
{

  if (steps>=0)
  {
    return true;
  
  }else 
  {
    return false;
  }
}

void setup()
{

  Serial.begin(9600);                                   //enable serial comunication
  Wire.begin();                                         //start i2C
  Wire.setClock(800000L);                               //fast clock

  pinMode(START_PROGRAM_PIN, INPUT);                   

  cli();                                                // disable interrupts for timers/counters setup

  //NOTE - TIMERS 0,1,3,4,5 share the same prescaler module


  //Setup for timer 3
  TCCR3A = 0;                                           // set TCCR registers controling timer operation mode
  TCCR3B = 0;
  TCNT3 = 0;                                            // set zero as counter value
  OCR3A = 0xff;                                         // set compare register value initially at max value
  TCCR3B |= (1 << WGM32);                               // turn on CTC (Clear timer on compare match) mode
  TCCR3B |= (0 << CS30) | (1 << CS31) | (0 << CS32);    // set prescaler value to 8
  TIMSK3 |= (0 << OCIE3A);                              // disable interrupts for this timer


  //Setup for timer 4
  TCCR4A = 0;                                           // set TCCR registers controling timer operation mode
  TCCR4B = 0;
  TCNT4 = 0;                                            // set zero as counter value
  OCR4A = 0xff;                                         // set compare register value initially at max value
  TCCR4B |= (1 << WGM42);                               // turn on CTC (Clear timer on compare match) mode
  TCCR4B |= (0 << CS40) | (1 << CS41) | (0 << CS42);    // set prescaler value to 8
  TIMSK4 |= (0 << OCIE4A);                              // disable interrupts for this timer


  //Setup for timer 5
  TCCR5A = 0;                                           // set TCCR registers controling timer operation mode
  TCCR5B = 0;
  TCNT5 = 0;                                            // set zero as counter value
  OCR5A = 0xff;                                         // set compare register value initially at max value
  TCCR5B |= (1 << WGM52);                               // turn on CTC (Clear timer on compare match) mode
  TCCR5B |= (0 << CS50) | (1 << CS51) | (0 << CS52);    // set prescaler value to 8
  TIMSK5 |= (0 << OCIE5A);                              // disable interrupts for this timer


  sei();                                                //enable global interrupts for normal operation

  //encoder_1.checkMagnet();                            //check for the magnet on all 3 encoders
  //encoder_2.checkMagnet();
  //encoder_3.checkMagnet();



}


//TIMER 3 Interrupt
ISR(TIMER3_COMPA_vect)
{
  if (!motor_1.motor_state)
  {
    digitalWrite(motor_1.stepPin, HIGH);
    motor_1.motor_state = 1;
  }
  else
  {
    digitalWrite(motor_1.stepPin, LOW);
    motor_1.motor_state = 0;
  }

  motor_1.steps_done = motor_1.steps_done + 1;                // increment number of steps already done

  if (motor_1.steps_done >= motor_1.steps_required)          // when number of steps done, reaches required number of steps, turn off stepping routine
  {
    motor_1.stopMove();
  }
}


//TIMER 4 Interrupt
ISR(TIMER4_COMPA_vect)
{
  if (!motor_2.motor_state)
  {
    digitalWrite(motor_2.stepPin, HIGH);
    motor_2.motor_state = 1;
  }
  else
  {
    digitalWrite(motor_2.stepPin, LOW);
    motor_2.motor_state = 0;
  }


  motor_2.steps_done = motor_2.steps_done + 1;        // increment number of steps already done

  if (motor_2.steps_done >= motor_2.steps_required)           // when number of steps done, reaches required number of steps, turn off stepping routine
  {
    motor_2.stopMove();
  }
}

//TIMER 5 Interrupt
ISR(TIMER5_COMPA_vect)
{
  if (!motor_3.motor_state)
  {
    digitalWrite(motor_3.stepPin, HIGH);
    motor_3.motor_state = 1;
  }
  else
  {
    digitalWrite(motor_3.stepPin, LOW);
    motor_3.motor_state = 0;
  }


  motor_3.steps_done = motor_3.steps_done + 1;                  // increment number of steps already done

  if ( motor_3.steps_done >= motor_3.steps_required)           // when number of steps done, reaches required number of steps, turn off stepping routine
  {
    motor_3.stopMove();
  }
}





void loop()
{
  program_start = digitalRead(START_PROGRAM_PIN);
  if (program_start !=0)
  {
    getProgram();
    decodeProgram();
    runProgram(); 
  }
  
}
