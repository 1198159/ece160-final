
#include <stdint.h>
#include "Energia.h"
#include <PID_v1.h>

//class for combined motor+encoder
class Motor
{
private:
  uint8_t aPin, bPin, slpPin, dirPin, pwmPin;
	uint32_t* eCount;
	uint8_t* eDir;
  double setPoint, input, output;
  PID* pidController;
public:
  Motor();
	void start(int slpPin, int dirPin, int pwmPin, int aPin, int bPin, uint32_t* count, uint8_t* dir);
	void setEnabled(bool enable);
	void setPower(double power);
  void setVelocity(double power);
	void update(void (*func)(double));
  void asLeft();
  void asRight();
  uint32_t getEncoderValue();
};
