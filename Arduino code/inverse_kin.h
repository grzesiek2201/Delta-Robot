
#ifndef inverse_kin_h
#define inverse_kin_h




//#include <Arduino.h>
#include <math.h>
#include <stdio.h>

class inverse_kin 
{
private: 

const float OUT_OF_RANGE_ANGLE_LOW = -0.5235987756;
const float OUT_OF_RANGE_ANGLE_HIGH = 1.5707963268;

// geometry of the robot
float sb = 375;  // length of the Side of the Base triangle in mm (sb)
float sp = 76; // length of the Side of the effector (Plate) triangle in mm (sp)
float Length = 200;  // length of the biceps in mm (L)
float length = 281;  // length of the forearm in mm (l)
float wb = 1.0 / 3 * sqrt(3) / 2 * sb;  // distance from the center of the Base to the side of the triangle
float ub = 2 * wb;  // distance from the center of the Base to the side of the triangle
float wp = 1.0 / 3 * sqrt(3) / 2 * sp;  // distance from the center of the effector (Plate) to the vertex of the triangle
float up = 2 * wp;  // distance from the center of the effector (Plate) to the vertex of the triangle
float a = wb - up;
float b = sp / 2 - sqrt(3) / 2 * wb;
float c = wp - 1.0 / 2 * wb;



// vectors for motor joints in base frame {B} (from base frame to the point where motors are mounted)
float B_1[3] = {0, -wb, 0};
float B_2[3] = {sqrt(3) / 2 * wb, 1.0 / 2 * wb, 0};
float B_3[3] = {-sqrt(3) / 2 * wb, 1.0 / 2 * wb, 0};

// vectors for effector joints Pi in effector frame {P}
float P_1[3] = {0, -up, 0};
float P_2[3] = {sp / 2, wp, 0};
float P_3[3] = {-sp / 2, wp, 0};

// list for the base vertices
float Bvx[4] = {-sb / 2, 0, sb / 2, -sb / 2};
float Bvy[4] = {-wb, ub, -wb, -wb};
float Bvz[4] = {0, 0, 0, 0};

public:

int calculations(float position_data[3], float fi_array[3]);
};

#endif
