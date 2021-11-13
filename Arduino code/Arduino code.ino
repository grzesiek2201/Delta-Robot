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
#define STEP_HIGH_1             PORTF |=  0b00000001; //lub 1 bit port C
#define STEP_LOW_1              PORTF &= ~0b00000001;
#define TIMER_INTERRUPT_1_ON    TIMSK3 |=  (1<<OCIE3A);
#define TIMER_INTERRUPT_1_OFF   TIMSK3 &= ~(1<<OCIE3A);

#define STEP_PIN_2         60
#define DIR_PIN_2          61
#define ENABLE_PIN_2       56
#define STEP_HIGH_2             PORTF |=  0b01000000; // lub bit 7 port c
#define STEP_LOW_2`             PORTF &= ~0b01000000;
#define TIMER_INTERRUPT_2_ON    TIMSK4 |=  (1<<OCIE4A);
#define TIMER_INTERRUPT_2_OFF   TIMSK4 &= ~(1<<OCIE4A);

#define STEP_PIN_3         46
#define DIR_PIN_3          48
#define ENABLE_PIN_3       62
#define STEP_HIGH_3             PORTL |=  0b00001000; // 3 bit port d
#define STEP_LOW_3              PORTL &= ~0b00001000;
#define TIMER_INTERRUPT_3_ON    TIMSK5 |=  (1<<OCIE5A);
#define TIMER_INTERRUPT_3_OFF   TIMSK5 &= ~(1<<OCIE5A);

#define START_PROGRAM_PIN  11 // PIN NA KTÓRY DAMY SYGNAŁ STARTU PROGRAMU 

#define MAX_NUM_OF_POINTS  100

#define MICROSTEPPING      16
#define STEPS_PER_REV      200

#define POINTS_DENSITY 4        //How many points for 1 cm of distance traveled 

#define MEMORY_SAFETY_MARIGIN 10  //[%]  allocate x% more memmory than aproximated 



//define global variables 
const double STEP_ANGLE = 360 / STEPS_PER_REV;
int program_length=0; 
int program_converted_length=0;
int program_start=0;
float previous_angles[3]={10,20,30};
float previous_position[3]= {0,0,-220};        
float speed_override =1;
float acc_override =1; 


///////////////////////  STRUCTURES AND CLASSES ///////////////////////

struct PositionData
{
  float abc[3]={0,0,0};                     //angle data [deg]
  float xyz[3]={0,0,0};                     //position data [mm] 
};

//Structure containing data about possible movement parameters of the motors 
struct MotionParam
{

  unsigned int min_interval=300;           //max speed - !!EXPERIMENTAL VALUE!!
  unsigned int acceleration_coef=8000;      //coefficient definig acceleration - !!EXPERIMENTAL VALUE!!

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
  float xyz[3]={0,0,0};             //x data [mm]
};

//Structure to contain movement point data after convertion
struct PointConverted
{
  unsigned int other_info [3];                //[0] - interpolation: 0=JOINT;  1=LINEAR;      [1] - movement mode - 0=accelerate; 1=mentain speed; 2=deaccelerate;    [2]- point of origin;  
  int steps[3]={0,0,0};                      //number of steps motor3 
  bool state_flag=false;                     //variable monitoring if efector already reached given point 
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
    int conv_point=0;         // currently moving to this point 
    unsigned int comp_reg_value=0;    //register value that timer compares to 
    unsigned int slope_len_iterator=0;      //iterator for measuring how many steps there are in slope 
    unsigned int interpolation=0;   //
    unsigned int movement_mode=0;   // accelerate / mentain speed / slow down 
    unsigned int movement=0;              
    unsigned int min_interval=0;            //top speed 
    unsigned int slope_len=0;

  

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
    void move (bool dir, int steps, int converted_point_num, unsigned int minimal_interval, unsigned int acceleration_coef, unsigned int current_interpolation, unsigned int current_movement_mode)
    {
      motion_done=false;
      steps_required = 2*steps;
      conv_point = converted_point_num; 
      interpolation = current_interpolation;
      movement_mode = current_movement_mode;
      comp_reg_value =  acceleration_coef;
      min_interval = minimal_interval;
      slope_len=0;
      unsigned int reg= comp_reg_value;
      
      while(1)          //calculate slope_len
      { 
        reg= reg - (2*reg) / (4 * slope_len + 1);
        slope_len++;
        if (min_interval > reg ) {break;}
      }        
      
      if (dir)                                                         // set dir pin state accordingly to provided direction variable
      {
        digitalWrite(dirPin, HIGH);
      }else
      {
        digitalWrite(dirPin, LOW);
      }

      switch (timerNumber)
      {
        case 3:
          cli();                          //disable interrupts for modifiaction
          TCNT3 = 0;                      // set zero as counter value
          OCR3A = comp_reg_value;    // set compare register value
          TIMSK3 |= (1 << OCIE3A);        // enable interrupts for this timer/counter
          sei();                          //enable interrupts for normal operation
          break;

        case 4:
          cli();
          TCNT4 = 0;
          TIMSK4 |= (1 << OCIE4A);
          OCR4A = comp_reg_value;
          sei();
          break;

        case 5:
          cli();
          TCNT5 = 0;
          TIMSK5 |= (1 << OCIE5A);
          OCR5A = comp_reg_value;
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


// download robot program from the computer 
void getProgram()
{ 

  // program_length=0;                //zero previous values 
  // program_converted_length=0;
  // free (Program);
  // free (ProgramConverted);          //free memory allocated for previous program 
  // // Set serial communication with Python program
  // // get number of points in robot program.
  // long int float_converter[4];      //shifting bits on float does not work :( 
  // long int float_temp_accumulator[3];

  // //komentarze po Polsku są robocze - do kasacji potem 

  // program_length= 20;                 //z komunikacji przychodzi taka wartość, załóżmy że masz taką długość programu gdzie dla uproszczenia każdy punkt to 3x float 
  //                                   //wiadomo więc że przyjdzie 20 x 3 floaty czyli 20x3x4 bajty 


  // byte myTransfer_packet_rxObj[240];      //dostajemy taką tablicę danych w bajtach 

  // if (program_length>= MAX_NUM_OF_POINTS )
  // {
  //   Serial.println("FATAL ERROR - TOO LONG PROGRAM");
  // } 
  // else 
  // {
  //   Program = new Point [program_length];   //assign memory accordingly to number of points 
  // }



  // for (int i=0; i<program_length; i++)      //process data for every point 
  // {

  //   for (int k=0; k<3; k++)                   // take floats in packets of 3 - that many floats is needed to fill one point 
  //   {

  //       for(int j=0; j<4; j++)                  // take bytes in packets of 4 - that many bytes is needed to combine into one float point of data 
  //       {
  //         float_converter[j]= (long int) myTransfer_packet_rxObj[12*i+4*k+j];  //from table of bytes, cast data onto long int variable. 
  //         //At one 'i' iteration, 12 data points will be consumed; respectively - at one 'k' iteration, 4 data points
  //       }
  //       float_converter[0] =  float_converter[0] << 24;      // most significant byte here  
  //       float_converter[1] =  float_converter[1] << 16;
  //       float_converter[2] =  float_converter[2] << 8;
  //       // float_converter[3] contains least significant byte, no shift needed 
  //       float_temp_accumulator[k] = float_converter[0] | float_converter[1] | float_converter[2] | float_converter[3];  //combine 4 bytes into one long int 
  //   }
 
  //   // check validity of data - interpolation must be 0,1 or 2, speed 0-100, acc 0-100 
  //   // if any data is invalid - signal an error and abort robot program download 
  //   Program[i].index_of_point=i;
  //   // Program[i].interpolation=NULL;
  //   // Program[i].speed=NULL;
  //   // Program[i].acc=NULL;
  //   Program[i].x= (float) float_temp_accumulator[0];
  //   Program[i].y= (float) float_temp_accumulator[1];
  //   Program[i].z= (float) float_temp_accumulator[2];  
  
  // }

}

//take downloaded robot program and convert it so its ready to be run 
void decodeProgram() 
{
  //
  //program_length=0;                //zero previous values //not this one 
  program_converted_length=0;
  //free (Program);
  free (ProgramConverted);          //free memory allocated for previous program 
  //

  int aprox_conv_prog_len=0;
  Serial.println("DECODE START");
  Serial.flush();
  aprox_conv_prog_len = calculateApproxConvertedProglength();                             //calculate approximated converted program length 
  Serial.flush();
  Serial.print("aprox_conv_prog_len:");
  Serial.println(aprox_conv_prog_len);
  Serial.flush();

  //ProgramConverted = new PointConverted [aprox_conv_prog_len*(1+(MEMORY_SAFETY_MARIGIN/100))];        //allocate memory on converted program pointer, program length approxiated 
  ProgramConverted = new PointConverted [aprox_conv_prog_len];        //allocate memory on converted program pointer, program length approxiated 


  for (int i=0; i<program_length; i++)                        // run functions (interpolation + kinematics + data transfer) for every user-defined  point in program 
  {
    if (Program[i].interpolation==0)              // interpolation 0 - joint 
    {
      Serial.flush();
      Serial.println("jointInterpolation in");
      jointInterpolation(i);
      Serial.flush();
      Serial.println("jointInterpolation out");
      Serial.flush();
    }
    else if (Program[i].interpolation==1)         // interpolation 1 - linear
    { 
      Serial.flush();
      Serial.println("linInterpolation in");
      linearInterpolation(i);
      Serial.flush();
      Serial.println("linInterpolation out");
      Serial.flush();
    }
  }
  Serial.flush();
  Serial.println("Program DECODED ");
  Serial.flush();
  delay(1000);
  for (int i=0; i<aprox_conv_prog_len; i++)
  {
    Serial.flush();
    Serial.print("INTERPOL: ");
    Serial.flush();
    Serial.println(ProgramConverted[i].other_info[0]);
    Serial.flush();
    delay(20);
    Serial.print("steps[0]:");
    Serial.println(ProgramConverted[i].steps[0]);
    Serial.flush();
    delay(20);
    Serial.print("steps[1]:");
    Serial.println(ProgramConverted[i].steps[1]);
    Serial.flush();
    delay(20);
    Serial.print("steps[2]:");
    Serial.println(ProgramConverted[i].steps[2]);
    Serial.flush();
    delay(20);
  }

}

void jointInterpolation(int p_index_number)        
{ 
  Serial.flush();
  Serial.println("joint in ");
  float motor_angles[3];
  float angular_distance[3];

  float distance=0;
  float buffer=0;
  int i = p_index_number;
  
  inverse_kin.calculations (Program[i].xyz, motor_angles);                        // do inverese kinematics and get the angle data from motor_angles array 
  calculateAngularDistance (angular_distance,motor_angles);

  for (int k=0; k<3; k++)                                                              
  {
    previous_angles[k] = motor_angles[k];
    previous_position[k] = Program[i].xyz[k];    
    buffer = angular_distance[k] * 3 * STEPS_PER_REV * MICROSTEPPING / 360; 
    ProgramConverted[program_converted_length].steps[k]= (int) buffer; 
  }
  ProgramConverted[program_converted_length].other_info[0] = Program[p_index_number].interpolation;
  ProgramConverted[program_converted_length].other_info[2] = p_index_number;
  program_converted_length++; 
}

void linearInterpolation(int p_index_number)
{
  int i = p_index_number;
  int estimated_num_of_points=0;
  float buffer=0;
  float distance=0;
  float vector_array[3];
  float temp_coordinates[3];
  float motor_angles[3];
  float angular_distance[3];
  int step_distance[3];
  int step_sum[3];
  int steps_to_be_done[3];
  
  
  distance= calculateDistanceLine(i);                                             //get the linear distance to target                                 
  estimated_num_of_points = (int) distance * POINTS_DENSITY /10;                  //calculate how many intermediate points should be created to achevie linear movement with given resolution 
  returnDirectionVectors (i, estimated_num_of_points, vector_array);              //basing on number of intermediate points, calculate length of unit vectors (between intermediate points)

  for (int j=0; j<estimated_num_of_points-1; j++)                                 //generate all but last intermediate points 
  {
    ProgramConverted[j].other_info[0] = Program[p_index_number].interpolation ;    //transfer interpolation data 
    ProgramConverted[j].other_info[2] = p_index_number;
    
    for (int k=0; k<3 ; k++)
    {
      temp_coordinates[k] = previous_position[k] + vector_array[k];                       //coordinates of point [j] =  actual position + j* unit vector of translation on each axis 
      previous_position[k]= temp_coordinates[k];
    }
    inverse_kin.calculations (temp_coordinates, motor_angles);                  // do inverese kinematics and get the angle data from motor_angles array 
    calculateAngularDistance (angular_distance, motor_angles);
    
    for (int k=0; k<3 ; k++)
    {
      previous_angles[k] =  motor_angles[k];
      buffer = angular_distance[k] * 3 * STEPS_PER_REV * MICROSTEPPING / 360; 
      step_distance[k]= (int) buffer;
      steps_to_be_done[k]+=step_distance[k];
    }
    assignStepDistance (step_distance);
    program_converted_length++;
  }

  // for the last point do different procedure- just assign data from oryginal target point 
  ProgramConverted[program_converted_length].other_info[0] = Program[p_index_number].interpolation ;    //transfer interpolation data 
  ProgramConverted[program_converted_length].other_info[2] = p_index_number;
  inverse_kin.calculations (Program[p_index_number].xyz, motor_angles); 
  calculateAngularDistance (angular_distance,motor_angles); 

  for (int k=0; k<3; k++)                                                             //calculate distance from start of the move to a given intermediate point in steps 
  {
    buffer = angular_distance[k] * 3 * STEPS_PER_REV * MICROSTEPPING / 360; 
    step_distance[k]= (int) buffer ;
    previous_angles[k] =  motor_angles[k];
    previous_position[k]= temp_coordinates[k];
    steps_to_be_done[k]+=step_distance[k];
  }
  assignStepDistance (step_distance);
  program_converted_length++;
  assignMovementModes(program_converted_length, steps_to_be_done, estimated_num_of_points);
}

void assignMovementModes(int converted_point_index, int steps_to_be_done[3], int estimated_num_of_points)
{
  calculateMotionOverride(converted_point_index);
  unsigned int min_interval = MotionParam.min_interval / speed_override; 
  unsigned int acceleration_coef = MotionParam.acceleration_coef / acc_override;  
  unsigned int reg=acceleration_coef;
  unsigned int slope_len=0;
  int step_sum[3]={0,0,0};
  while(1)          //calculate slope_len
  { 
    reg= reg - (2*reg) / (4 * slope_len + 1);
    slope_len++;
    if (min_interval > reg ) {break;}
  }

  for (int i=0; i<estimated_num_of_points; i++)
  {
    for (int k=0; k<3; k++)
    {
      step_sum[k]+=ProgramConverted[program_converted_length-estimated_num_of_points+i].steps[k]
      if(step_sum[k]<=slope_len)
      {
        ProgramConverted[program_converted_length-estimated_num_of_points+i].other_info[1] = 0; //accelerate
      }else if (step_sum[k]>=steps_to_be_done[k]-slope_len)
      {
        ProgramConverted[program_converted_length-estimated_num_of_points+i].other_info[1] = 2; //slow down
      }else 
      {
        ProgramConverted[program_converted_length-estimated_num_of_points+i].other_info[1] = 1; //mentain speed 
      }
    }
  }

}


void calculateMotionOverride(int converted_point_num)
{
  unsigned int original_point=0;
  original_point = ProgramConverted[converted_point_num].other_info[2];
  speed_override = Program[original_point].speed /100 * MotionParam.user_defined_speed_override * MotionParam.safety_override ;
  acc_override = Program[original_point].acc /100 * MotionParam.user_defined_acc_override *  MotionParam.safety_override ;
  if (speed_override>1){speed_override=1;}
  if (speed_override<=0){speed_override=0,001;}
  if (acc_override>1){acc_override=1;}
  if (acc_override<=0){acc_override=0,001;}

}

void assignStepDistance (int step_distance[3])
{
  for (int k=0; k<3; k++)
  {
    ProgramConverted[program_converted_length].steps[k] = step_distance[k];
  }
}

void calculateAngularDistance (float angular_distance[3], float motor_angles[3])
{
  for(int i=0; i<3; i++)
  {
  angular_distance[i] = motor_angles[i] - previous_angles[i];
  }
  
}

void returnDirectionVectors (int i, int num_of_intermediate_points, float vector_array[3])
{
  float diff_xyz[3]={0,0,0};

  for (int k=0; k<3; k++)
  {
    if (i==0)
    {
    diff_xyz[k] = Program[i].xyz[k] - previous_position[k];
    }
    else 
    {
    diff_xyz[k] = Program[i].xyz[k] - Program[i-1].xyz[k];
    }
    vector_array[k] = diff_xyz[k] / num_of_intermediate_points;
  }

}

float calculateDistanceLine (int i)
{
  float distance=0;
  float diff_xyz[3]={0,0,0};

  for (int k=0; k<3; k++)
  {
    if (i==0)
    {
      diff_xyz[k] = Program[i].xyz[k] - previous_position[k];
    }else 
    {
      diff_xyz[k] = Program[i].xyz[k]- Program[i-1].xyz[k];
    }
  } 

  distance= sqrt ( sq(diff_xyz[0]) + sq(diff_xyz[1])  + sq(diff_xyz[2]));
  Serial.flush();
  Serial.print("i:");
  Serial.flush();
  Serial.println(i);
  Serial.flush();
  Serial.print("distance:");
  Serial.flush();
  Serial.println(distance);
  return distance;
}

int calculateApproxConvertedProglength()
{
  int sum=0;
  float distance=0;
  int num_of_points=0;

  for (int i=0; i<program_length; i++)
  {
    if (Program[i].interpolation==0)              // interpolation 0 - joint 
    {
      sum++;
    }
    else if (Program[i].interpolation==1)         // interpolation 1 - linear
    {
      distance= calculateDistanceLine(i);
      num_of_points = (int) distance * POINTS_DENSITY /10 ;   
      sum = sum + num_of_points; 
    }
  }
  return sum;
}

void getPos()
{
  // Multiplexer.selectChannel(0);                 // set channel on multiplexer, aiming at given encoder channel 
  // PositionData.a = encoder_1.getAngleDeg();     // get angle data from given encoder 
  // Multiplexer.selectChannel(1);
  // PositionData.b = encoder_2.getAngleDeg();
  // Multiplexer.selectChannel(2);
  // PositionData.c = encoder_3.getAngleDeg();
}

// execute decoded program 
void runProgram()
{
  int points_already_done=0;
  while (true)
  {

    //    Things to do while program is running - check encoderes, check coms, etc. 
    //
    //    !! stuff can be done IF ONLY it does not take too much time to complete !!
    //    It is crucial that this code completes fast so that there is not too much delay between motor state checks 
    //

    if (points_already_done==0 || ProgramConverted[points_already_done-1].state_flag==true)           //go to next point if 1) its a first point  or 2) previous point was reached 
    {
      move(points_already_done);
      points_already_done++;
    }
  
    if (points_already_done>=program_converted_length)                                                //if all the points were reched, break from the program running mode
    {
    break;
    }

  }
}

// Move effector to position 
void move(int converted_point_index )
{
  int i = converted_point_index;

  bool dir[3];  
  for (int k=0; k<3; k++)
  {
    dir[k]=checkDir(ProgramConverted[i].steps[k]);
  }
  
  calculateMotionOverride(i);
  unsigned int min_interval = MotionParam.min_interval / speed_override; 
  unsigned int acceleration_coef = MotionParam.acceleration_coef / acc_override;  
  unsigned int current_interpolation = ProgramConverted[converted_point_index].other_info[0];
  unsigned int current_movement_mode = ProgramConverted[converted_point_index].other_info[1];
  
  motor_1.move(dir[0], ProgramConverted[i].steps[0], i, min_interval, acceleration_coef, current_interpolation, current_movement_mode);         //set the movement of the motors
  motor_2.move(dir[1], ProgramConverted[i].steps[1], i, min_interval, acceleration_coef, current_interpolation, current_movement_mode);
  motor_3.move(dir[2], ProgramConverted[i].steps[2], i, min_interval, acceleration_coef, current_interpolation, current_movement_mode);
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
  
  Serial.begin(115200);                                   //enable serial comunication
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


  program_length=2;
  Program = new Point [program_length];
  ProgramConverted = new PointConverted [1];
  program_converted_length =1;
  
  ProgramConverted[0].other_info [0] = 0;
  ProgramConverted[0].other_info [1] = 0;
  ProgramConverted[0].other_info [2] = 0;                //[0] - interpolation: 0=JOINT;  1=LINEAR;      [1] - movement mode - 0=accelerate; 1=mentain speed; 2=deaccelerate;    [2]- point of origin;  
  ProgramConverted[0].steps[0]=1000;   
  ProgramConverted[0].steps[1]=1000;
  ProgramConverted[0].steps[2]=1000;                   //number of steps motor3 
  ProgramConverted[0].state_flag=false; 

  Program[0].index_of_point=0;         //point number in the program 
  Program[0].interpolation=0;          //0=JOINT  1=LINEAR ... 2=CIRCULAR if implemented 
  Program[0].speed=100;                   // [0-100%]
  Program[0].acc=100;                     // [0-100%]
  Program[0].xyz[0]=0;                     //x data [mm] 
  Program[0].xyz[1]=0; 
  Program[0].xyz[2]=-180;

  Program[1].index_of_point=1;         //point number in the program 
  Program[1].interpolation=0;          //0=JOINT  1=LINEAR ... 2=CIRCULAR if implemented 
  Program[1].speed=100;                   // [0-100%]
  Program[1].acc=100;                     // [0-100%]
  Program[1].xyz[0]=0;                     //x data [mm] 
  Program[1].xyz[1]=0; 
  Program[1].xyz[2]=-240;  

}


//TIMER 3 Interrupt
ISR(TIMER3_COMPA_vect)
{
  if (!motor_1.motor_state)
  {
    STEP_HIGH_1;
    motor_1.motor_state = 1;
  }
  else
  {
    STEP_LOW_1;
    motor_1.motor_state = 0;
  }
    
  motor_1.steps_done ++;                                                          // increment number of steps already done

  if (motor_1.steps_done >= motor_1.steps_required)                               // when number of steps done, reaches required number of steps, turn off stepping routine
  {
    motor_1.stopMove();
  }

  if (motor_1.interpolation==0)                                                   //joint interpolation 
  {
    if (motor_1.steps_done < motor_1.slope_len)                                   //should accelerate 
    {
      motor_1.movement=0;
    } else if (motor_1.steps_done > motor_1.steps_required - motor_1.slope_len)   //should begin slowing down 
    {
      motor_1.movement=2;
    } else                                                                        //just mentain speed 
    {
      motor_1.movement=1;
    }                                 

  }
  else if (motor_1.interpolation==1)                            //linear interpolation  movement_mode  0=accelerate / 1=mentain speed / 2=slow down
  {
    if (motor_1.movement_mode==0)                               //linear interpolation and accelearate 
    {
      if (motor_1.steps_done < motor_1.slope_len)               //accelerating phase 
      {motor_1.movement=0;} else
      {motor_1.movement=1;}                                     //stedy speed phase
    }else if (motor_1.movement_mode==1)
    {
      motor_1.movement=1;                                       //linear interpolation and this point should just mentain speed 
    }else if (motor_1.movement_mode==2)                         //linear interpolation and slow down                                           
    {
      if (motor_1.steps_done > motor_1.steps_required - motor_1.slope_len)           //slow down phase 
      {motor_1.movement=2;} else
      {motor_1.movement=1;}                                      //stedy speed phase
    }
  }

  
  if (motor_1.movement==0)                           //accelerate
  { 
    motor_1.comp_reg_value = motor_1.comp_reg_value - (2*motor_1.comp_reg_value) / (4 * motor_1.slope_len_iterator + 1);
    motor_1.slope_len_iterator++;
    TIMER_INTERRUPT_1_OFF;                                                                      //disable interrupts for modifiaction
    OCR3A = motor_1.comp_reg_value;                                            // set new compare register value                                       
    TIMER_INTERRUPT_1_ON;                                                                        //enable interrupts for normal operation

  }else if (motor_1.movement==2)                     //slow down 
  {
    motor_1.comp_reg_value = (motor_1.comp_reg_value *(4 * motor_1.slope_len_iterator +1 )) / (4*motor_1.slope_len_iterator-1); 
    motor_1.slope_len_iterator--;
    TIMER_INTERRUPT_1_OFF;                                                                       //disable interrupts for modifiaction
    OCR3A = motor_1.comp_reg_value;                                            // set new compare register value
    TIMER_INTERRUPT_1_ON;
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
  
  //dir,    steps,    converted_point_num,    minimal_interval,     acceleration_coef,    current_interpolation,    current_movement_mode
  // -        -               0               small  = fast            small = quick                0                      whatever 
  
  motor_1.move(0,6400,0,100,5000,0,0);  
  delay(20000);
  
}
