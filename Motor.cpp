#include "stdlib.h"
#include "SimpleRSLK.h"
#include "Motor.h"

//static variables for interrupts
uint32_t left_cnt = 0;
uint32_t right_cnt = 0;
uint8_t left_wheel_dir = 0;
uint8_t right_wheel_dir = 0;

//PID terms for the velocity PID;
const double P = 1, I = 0, D = 0;

//create motor
Motor::Motor(){}
void Motor::start(int s, int d, int p, int a, int b, uint32_t* count, uint8_t* dir){
  aPin = a;
  bPin = b;
  slpPin = s;
  dirPin = d; 
  pwmPin = p;
  eCount = count;
  eDir = dir;
  PID controller(&input, &output, &setPoint, P,I,D, DIRECT);
  pidController = &controller;

  input = 0; 
  setPoint = 0;

  pidController->SetMode(AUTOMATIC);
  pidController->SetOutputLimits(-100, 100);
  pinMode(slpPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
//  pinMode(aPin, INPUT_PULLDOWN);
//  pinMode(bPin, INPUT_PULLDOWN);

  setEnabled(true);


}
//left trigger method
void triggerLeft() {
  left_cnt++;

  /* The wheel direction can be determined by the second encoder pin */
  left_wheel_dir = digitalRead(ENCODER_ELA_PIN);
}
//right trigger method
void triggerRight() {
  right_cnt++;

  /* The wheel direction can be determined by the second encoder pin */
  right_wheel_dir = digitalRead(ENCODER_ERA_PIN);
}

//make this motor left
void Motor::asLeft(){
  start(MOTOR_L_SLP_PIN, MOTOR_L_DIR_PIN, MOTOR_L_PWM_PIN, ENCODER_ELA_PIN, ENCODER_ELB_PIN, &left_cnt, &left_wheel_dir);
//  attachInterrupt(digitalPinToInterrupt(ENCODER_ELB_PIN),triggerLeft,RISING);  
}
//make this motor right
void Motor::asRight(){
  start(MOTOR_R_SLP_PIN, MOTOR_R_DIR_PIN, MOTOR_R_PWM_PIN, ENCODER_ERA_PIN, ENCODER_ERB_PIN, &right_cnt, &right_wheel_dir);
//  attachInterrupt(digitalPinToInterrupt(ENCODER_ERB_PIN),triggerRight,RISING);

}
//pid stuff
double pastTime, currTime;
uint32_t pastEncoder, currEncoder;
//Serial1.begin(57600);
void Motor::update(void (*func)(double)){
  pastTime = currTime;
  currTime = micros()/1000000.0;
  double deltaTime = 1;// currTime-pastTime;
  pastEncoder = currEncoder;
  currEncoder = getEncoderValue();
  double deltaEncoder = 1;// currEncoder-pastEncoder;
  if(deltaTime != 0) input = deltaEncoder/deltaTime;
  pidController->Compute();
  if(func != NULL){
    func(deltaTime);
    func(deltaEncoder);
    func(input);
    func(output);
  }
//  setPower(output);
  
}
//get encoder value
uint32_t Motor::getEncoderValue(){
  return (*eCount)*(*eDir ? 1 : -1);
}
//enable or disable motor
void Motor::setEnabled(bool enable){
	digitalWrite(slpPin, enable);
}
//set motor velocity
void Motor::setVelocity(double power){
  setPoint = power;
}
//set motor raw power
void Motor::setPower(double power){
	int p = abs(power*255.0);
	digitalWrite(dirPin, power < 0);
	analogWrite(pwmPin, p);
}
