
#include "multiplexer.h"



Multiplexer :: Multiplexer(int multiplexer_adress)
{
   adress = multiplexer_adress;
};

void Multiplexer ::  selectChannel (int chanell_index)
{
  Wire.beginTransmission(adress);         // begin transmition to multiplexer
  Wire.write(1 << chanell_index);         // send index to select given channel 
  Wire.endTransmission();
};