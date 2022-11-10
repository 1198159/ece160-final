/* Section 3 Team 5 ARG code for the Rosie Rescue competition
 *  
 * By Alex, Gavin, and Ronin
 * 
 * This code was designed to allow high levels of automation without adding tons of sensors, so advanced line following and encoders were required
 * 
 * This file has the bulk of the code for multiple reasons. 
 * The main reason is OOP. Our code has custom objects for tons of components, and to use multpile files would require passing around a ton of variables. 
 * There are separate files for the custom Motor and LineSensor objects which add really nice functionality, but don't contain game-specific commands
 * 
 * To compile this code you need the following libraries:
 * Servo
 * the SimpleRSLK package
 * PS2X_lib
 * TinyIRRemote
 * PID_v1
 * 
 * The version control of this code was done via a git server hosted directly on Onedrive, due to initial miscommunication on what was allowed/disallowed and it was fun to do
 * 
 * 
 */
 
#include "SimpleRSLK.h"
#include "Servo.h"
#include "ROMI_MOTOR_POWER.h"
#include "PS2X_lib.h"
#include "LineSensor.h"
#include "Motor.h"
#include "TinyIRremote.h"
#include "PID_v1.h"

//get sign of a number
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

//controller pins
const int BROWN = 15, ORANGE = 14, BLUE = 34, YELLOW = 35;

//controller object
PS2X controller;
//servo object
Servo gripper;
//motor objects
Motor leftMotor, rightMotor;
//linesensor object
LineSensor sensor;
IRsend sendIR; //Object that is required to transmit (Tx)

//pid values
double setPoint, input, output;
//p.08 d.005
const double P = 0.12, I = 0.0, D = 0.014, SPEED_UP = 0.5, MIN_SPEED = 0.3;
//tolerance to detect if a line ends
const double LINE_END_TOLERANCE = 0.2;

PID follower(&input, &output, &setPoint, P, I, D, DIRECT);

//LEDs for artistic effect in tunnel
const int SPECIAL_LED = 19;
const int lightLedPin = 17;

//current value of the line sensor
double lineVal;

//IR pins
const int irLedPin = 37;
const int irReadPin = A12;

//light sensor pins and config
const int lightSensor = A9;
int lightLevel;
int calLight = 450;
const int lightTolerance = 100;

//setup function
void setup() {
  //start serial
  Serial1.begin(57600);
  Serial.begin(9600);

  //wait some time
  delayMicroseconds(2000000);

  //setup LEDS
  pinMode(lightSensor, INPUT);
  pinMode(lightLedPin, OUTPUT);

  //start controller
  handleError(controller.config_gamepad(BLUE, ORANGE, YELLOW, BROWN, true, true));
  setupRSLK();

  //setup motors/servos
  gripper.attach(SRV_0); //This is pin 38, from RSLK_Pins.h
  leftMotor.asLeft();
  rightMotor.asRight();
  setupEncoder(ENCODER_ELA_PIN, ENCODER_ELB_PIN, ENCODER_ERA_PIN, ENCODER_ERB_PIN);
  sensor.calibrate();

  //PID stuff
  follower.SetMode(AUTOMATIC);
  follower.SetOutputLimits(-100, 100);
  setPoint = 0;
  
  openClaw();

}
//states the robot can be in
enum class State {
  CONTROLLER, SERIAL, LINE_FOLLOW, CALIBRATE, SEND_IR, AUTO
};
//states for the autonomous programs
const int AUTO_INIT = 0, AUTO_PICKUP = 1, AUTO_KILL_1 = 2, AUTO_SCORE_1 = 3, AUTO_KILL_2 = 4, AUTO_SCORE_2 = 5, AUTO_KILL_3 = 6, AUTO_RETURN = 7, AUTO_SCORE_3 = 8;

//current state
State state = State::CONTROLLER;
//current auto state
int autoState = 0;
//max drivespeed value for driver control
double driveSpeed = 0.5;

//choose between multiple states
void pickState() {
  controller.read_gamepad();
  if (Serial1.available()) state = State::SERIAL;
  else if (controller.ButtonPressed(PSB_START)) state = State::CONTROLLER;
  else if (controller.ButtonPressed(PSB_SQUARE)) state = State::CALIBRATE;
  else if (controller.ButtonPressed(PSB_TRIANGLE)) state = State::LINE_FOLLOW;
  else if (controller.ButtonPressed(PSB_SELECT)) state = State::SEND_IR;
  else if (controller.ButtonPressed(PSB_L2)) driveSpeed = 1.0;
  else if (controller.ButtonPressed(PSB_R2)) driveSpeed = 0.5;
  else if (controller.ButtonPressed(PSB_PAD_UP)) state = State::AUTO;
  else if (controller.ButtonPressed(PSB_PAD_LEFT)) autoState = max(AUTO_INIT, autoState-1);
  else if (controller.ButtonPressed(PSB_PAD_RIGHT)) autoState = min(AUTO_SCORE_3, autoState+1);
}

//main loop function with state machine
bool stp = false;
void loop() {
  readLights();
  pickState();
  switch (state) {
    //control robot from serial monitor
    case State::SERIAL:
      {
        if (!Serial1.available()) break;
        char i = Serial1.read();

        if (i == 'z') openClaw();
        if (i == 'x') closeClaw();
        double fb = 0, lr = 0;
        if (i == 'w' || i == 'e' || i == 'q') fb -= 0.3;
        if (i == 's') fb += 0.3;
        if (i == 'a' || i == 'q') lr -= 0.15;
        if (i == 'd' || i == 'e') lr += 0.15;
        arcadeDrive(lr, fb);
      }
      break;
    //calibration state
    case State::CALIBRATE:
      calibrateLine();
      break;
   //the autonomous functions
    case State::AUTO:
      switch (autoState) {
        case AUTO_INIT:
          //calibrate for a time
          for (int i = 0; i < 100; i++) calibrateLine();
          autoState++;
          break;
        case AUTO_PICKUP:
          //sequence for the auto pickup of the weapon and healthpack
          drive(-22);
          turn(90);
          pickup();
          turn(-90);
          drive(-14);
          turn(90);
          pickup();
          turn(-90);
          autoState++;
          break;
        case AUTO_KILL_1:
          //sequence to kill the first monster
          lineFollowDead(true);
          turn(90);
          lineFollowDist(32, 0);
          turn(90);
          lineFollowDist(60, 0);
          kill();
          autoState++;
          break;
         
        case AUTO_SCORE_1: 
          //score the first rosie
          lineFollowDead(true);
          turn(-90);
          scoreRosie(0);
          autoState++;
          break;
        case AUTO_KILL_2:
          //kill second monster
          passCorners(2, 90);
          kill();
          autoState++;
          break;
        case AUTO_SCORE_2:
          //score second rosie
          scoreRosie(1);
          autoState++;
          break;
        case AUTO_KILL_3:
          //kill third monster
          passCorners(4, 90);
          kill();
          autoState++;
          break;
        case AUTO_RETURN:
          //place blocks in way
          passCorners(2, -90);
          turn(90);
          drive(-20);
          openClaw();
          autoState++;
          state = State::CONTROLLER;
          break;
        case AUTO_SCORE_3:
          //score last rosie
          scoreRosie(2);
          autoState = 0;
          state = State::CONTROLLER;
          break;
      }
    //driver control state
    case State::CONTROLLER:
      {
        driverControl();
      }
      break;
    //line follow test state
    case State::LINE_FOLLOW:
      lineFollow();
      Serial.print(setPoint-input);

      break;
      //Ir test state
    case State::SEND_IR:
      sendIRSignal(0xA5, 0xC3);
      delay(100);

      sendIRSignal(160, 123);
      delay(100);

      sendIRSignal(160, 143);
      delay(100);
      break;
  }

}
//function to force cancel autonomous
bool isStopRequested(){
  controller.read_gamepad();
  bool b = controller.ButtonPressed(PSB_PAD_DOWN);
  if(b){
    state = State::CONTROLLER;
    stp = true;
  }
  return stp;
}
//calibrate the line sensor and the photoresistor
void calibrateLine() {
  sensor.calibrate();
  Serial1.println("calibrating");
  calLight = (analogRead(lightSensor) - lightTolerance); //Also calibrating the light in the room for the ledlight

}
//driver controlled functions for driving the robot
void driverControl() {
    tankDrive(-scaleStick(controller.Analog(PSS_RY)), scaleStick(controller.Analog(PSS_LY)));
    //  arcadeDrive(scaleStick(controller.Analog(PSS_LY)), scaleStick(controller.Analog(PSS_RX)));
    if (controller.ButtonPressed(PSB_L1)) openClaw();
    if (controller.ButtonPressed(PSB_R1)) closeClaw();
    stp = false;
}
//send signals to pick up healthpack and weapon
void pickup() {
  for (int i = 0; i < 5; i++) {
    sendIRSignal(160, 123);
    delay(100);

    sendIRSignal(160, 143);
    delay(100);
  }
}
//line follow for a certain amount of corners
void passCorners(int corners, int turnAmt) {
    if(isStopRequested()) return;
  for (int i = 0; i < corners; i++) {
    lineFollowDead(true);
    turn(turnAmt);
  }
  if (corners != 0) lineFollowDead(true);

}

//score the rosie in the extraction zone sequence
void scoreRosie(int ros) {
  if(isStopRequested()) return;
  closeClaw();
  lineFollowBlocking();
  delay(200);
  int trn = ros == 0 ? -40 : ros == 1 ? 40 : 0; 
      drive(-2);
  if(ros < 2){
  turn(trn);
  drive(-2);
  openClaw();
  drive(2);
   turn(sgn(trn)*(180-abs(trn)));
  }else{
    openClaw();
  }
}
//send the signals to kill a monster with IR and drive to prepare to pick up rosie
void kill() {
    if(isStopRequested()) return;
  for (int i = 0; i < 5; i++) {
    sendIRSignal(0xA5, 0xC3);
    delay(100);
  }
  drive(-3);
  for (int i = 0; i < 5; i++) {
    sendIRSignal(0xA5, 0xC3);
    delay(100);
  }

  turn(180);
  state = State::CONTROLLER;
}

//low level function to send a specific IR signal
void sendIRSignal(int address, int command) {
  IRData IRmsg; //Same object required for both Tx and Rx
  sendIR.begin(irLedPin, true, GREEN_LED);
  IRmsg.protocol = NEC; //use this protocol
  IRmsg.address = address; //Tx address (can be in decimal)
  IRmsg.command = command; //Tx command (can be in decimal)
  IRmsg.isRepeat = false; //Sends REPEAT instead of original command (don’t use)
  sendIR.write(&IRmsg); //Sends the data through the IR LED connected to IR_TRX_PIN
}
//stuff for detecting if in tunnel
int accum = 0;
boolean readLights() {
  lightLevel = analogRead(lightSensor);
//  Serial1.print("The light level is: ");
//  Serial1.print(lightLevel);
  if (lightLevel < calLight) {
    digitalWrite(lightLedPin, HIGH);
    if (accum == 100) {
      digitalWrite(SPECIAL_LED, LOW);
      accum = 0;
    } else {
      if (accum == 50) digitalWrite(SPECIAL_LED, HIGH);
      accum++;
    }
  }
  else {
    digitalWrite(SPECIAL_LED, LOW);
    digitalWrite(lightLedPin, LOW);
  }
  return lightLevel < calLight;

}
//basic function for line following
double lastInput = 0;
void lineFollow() {
  if(isStopRequested()) return;
  input = sensor.getValue();
  follower.Compute();
  //  Serial1.println(input);
  if (input > -0.99) lastInput = input;
  if (input < -0.99) {
    //    if(!readLights()){
    if (lastInput > LINE_END_TOLERANCE) arcadeDrive(0.15, 0);
    else if (lastInput < -LINE_END_TOLERANCE) arcadeDrive(-0.15, 0);
    else arcadeDrive(0, 0);
    return;
  }
  Serial1.print(output);
  Serial1.print("    ");
  arcadeDrive( -output, MIN_SPEED);

}
//line follow continuously and block main loop
void lineFollowBlocking() {
  lineFollow();
  delay(10);
  while ((input > -0.99 || abs(lastInput) > 0.2) && !isStopRequested()) {
    readLights();
    lineFollow();
  }

}
//follow a line, but dont turn automatically, repeat determines if the function blocks or not
void lineFollowDead(bool repeat) {
  if(isStopRequested()) return;
  input = sensor.getValue();
  follower.Compute();
  Serial1.println(input);
  if (input < -0.99) {
    arcadeDrive(0, 0);
    return;
  }
  Serial1.print(output);
  Serial1.print("    ");
  arcadeDrive( -output, 0.3);
  if (repeat) {
    delay(5);
    readLights();
    lineFollowDead(true);
  }
}

//line follow until encoders get to a certain distance
void lineFollowDist(int amt, double turn) {
  amt *= 40;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  int enc = 0;
  while (abs(enc) < abs(amt) && !isStopRequested()) {
    
    enc = getEncoderLeftCnt();
    lineFollowDead(false);
    if (input < -0.99) {
      if (turn == 0) break;
      arcadeDrive(turn, 0);
    }
  }
  arcadeDrive(0.0, 0.0);
}

//scale the stick to make robot more drivable
const double kStatic = 0.05;
double scaleStick(double value) {
  double s = map(value, 0, 255, -100, 100) / 100.0;
  s = s * s * s;
  return (s + sgn(s) * (kStatic/driveSpeed)) * (1 - (kStatic/driveSpeed)) * driveSpeed;
}

//open claw
void openClaw() {
  Serial1.println("opening claw");
  gripper.write(10);
}
//close claw
void closeClaw() {
  Serial1.println("closing claw");
  gripper.write(90);
}
//turn the robot to a certain amt of degrees (roughly)
void turn(int deg) {
  deg *= -1.8;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
    
  int enc1 = 0, enc2 = 0;
  while ((abs(enc1) < abs(deg) || abs(enc2) < abs(deg)) && !isStopRequested()) {
    enc1 = getEncoderLeftCnt();
    enc2 = getEncoderRightCnt();
    if (abs(enc1) < abs(deg)) setLeftPower(sgn(deg) * -0.2);
    if (abs(enc2) < abs(deg)) setRightPower(sgn(deg) * -0.2);
  }
  arcadeDrive(0.0, 0.0);
  delay(200);

}
//drive the robot to a certain amt of inches (roughly)

void drive(int inch) {
  inch *= 40;
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int enc1 = 0, enc2 = 0;
  while ((abs(enc1) < abs(inch) || abs(enc2) < abs(inch)) && !isStopRequested() ) {
    enc1 = getEncoderLeftCnt();
    enc2 = getEncoderRightCnt();
    if (abs(enc1) < abs(inch)) setLeftPower(sgn(inch) * -(abs(enc2) > abs(enc1) ? 0.25 : 0.2));
    if (abs(enc2) < abs(inch)) setRightPower(sgn(inch) * (abs(enc1) > abs(enc2) ? 0.25 : 0.2));

  }
  arcadeDrive(0.0, 0.0);
  delay(200);
}

//arcade drive function
void arcadeDrive(double x, double y) {
  tankDrive(-y - 2 * x, y - 2 * x);
}
//tank drive function
void tankDrive(double lp, double rp) {
  //printDriveState(lp, rp);
  setLeftPower(lp);
  setRightPower(rp);
}
void printDriveState(double lp, double rp){
    if (lp > 0 && rp > 0) Serial1.println("spin left");
    else  if (lp < 0 && rp < 0) Serial1.println("spin right");
    else if (lp > 0 && rp < 0) Serial1.println("drive forward");
    else if (lp < 0 && rp > 0) Serial1.println("drive backward");
    else if ((lp == 0 && rp > 0) || (rp == 0 && lp > 0)) Serial1.println("spin left drive");
    else if ((lp == 0 && rp < 0) || (rp == 0 && lp < 0)) Serial1.println("spin right drive");
    else Serial1.println("stop");
}
//set left motor power from -1.0 to 1.0
void setLeftPower(double power) {
  leftMotor.setPower(-power);
}

//set right motor power from -1.0 to 1.0
void setRightPower(double power) {
  rightMotor.setPower(power);
}

//handle controller error
void handleError(int error) {
  if (error == 0) {
    Serial1.println("Found Controller, configured successful");
    Serial1.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial1.println("holding L1 or R1 will print out the analog stick values.");
    Serial1.println("Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial1.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  else if (error == 2)
    Serial1.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  else if (error == 3)
    Serial1.println("Controller refusing to enter Pressures mode, may not support it. ");
  else Serial1.println("Unknown Error");
}
