#include <stdlib.h>
#include "LineSensor.h"
#include "SimpleRSLK.h"
//class for line sensor
LineSensor::LineSensor() {
  clearMinMax(sensorMinVals,sensorMaxVals);
}
//calibrate line sensor
void LineSensor::calibrate() {
  readLineSensor(sensorVals);
  setSensorMinMax(sensorVals, sensorMinVals, sensorMaxVals);
}
//get line sensor values
double LineSensor::getValue() {
  readLineSensor(sensorVals);
  readCalLineSensor(sensorVals, calibVals, sensorMinVals, sensorMaxVals, lineMode);
  return map(getLinePosition(calibVals, lineMode), 0, 7000, -1000, 1000)/1000.0;
}
