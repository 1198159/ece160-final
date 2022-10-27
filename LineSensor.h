#include "Energia.h"
//class for line sensor
class LineSensor
{
  //member variables
  private:
    uint16_t sensorVals[8];
    uint16_t calibVals[8];
    uint16_t sensorMinVals[8];
    uint16_t sensorMaxVals[8];
    uint8_t lineMode = 0; // 0 is for black line on white, 1 is for vice versa
   
  public:
    LineSensor();
    void calibrate();
    double getValue();
};
