#ifndef multiplexer_h
#define multiplexer_h

#include <math.h>
#include <stdio.h>
#include <Wire.h>

class Multiplexer 
{
private: 

int adress=NULL;

public:

Multiplexer(int multiplexer_adress);
void selectChannel (int chanell_index);

};

#endif
