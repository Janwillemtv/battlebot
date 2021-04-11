#include <PS4Controller.h>
#include <ESP32Servo.h>

#include <stdio.h>

#define MAX_OUT_CHARS 64  //max nbr of characters to be sent on diagnostic demand

#define MOTOR_ENABLE 32
#define INA 33
#define INB 25



//### CONFIG ######################
bool DIAGNOSTICS = true;
int MAXLEFT = 40;
int MAXRIGHT = 140;
int MAXSPEED = 255;

enum stMode {
  TWOWHEEL,
  FOURWHEEL,
  CRAB
};


//### INIT ########################
stMode steeringMode = TWOWHEEL;

int speedForward = 0;
int speedBackward = 0;
int steering = 0;
int deathWheel = 0;

bool optionDebounce = false;
bool triangleDebounce = false;

bool engageBrake = false;

// Steering
Servo frontLeft;
Servo frontRight;  
Servo rearLeft;  
Servo rearRight;    
// 16 servo objects can be created on the ESP32

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33 

void setup() {
  Serial.begin(115200);
  PS4.begin("B8:E8:56:41:5F:95");
  Serial.println("Ready.");

  // ##### STEERING
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // using default min/max of 1000us and 2000us
  // different servos may require different min/max settings
  // for an accurate 0 to 180 sweep
  int minPeriod = 500; // 1000us
  int maxPeriod = 2400; // 2000us
  
  frontLeft.setPeriodHertz(50);
  frontLeft.attach(13, minPeriod, maxPeriod);

  frontRight.setPeriodHertz(50);
  frontRight.attach(14, minPeriod, maxPeriod);

  rearLeft.setPeriodHertz(50);
  rearLeft.attach(12, minPeriod, maxPeriod);

  rearRight.setPeriodHertz(50);
  rearRight.attach(27, minPeriod, maxPeriod);

  //### DRIVE
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  
}

//### LOOP #########################
void loop() {
  // check connection to controller
  if (PS4.isConnected()) {
    checkController();
  }else{
    // safety wehn connection drops
    speedForward = 0;
    speedBackward = 0;
    steering = 0;
    deathWheel = 0;
    motorStop();
  }

  //### STEERING
  int steeringAngle = map(steering, -127, 127, MAXLEFT, MAXRIGHT);
  int mirrorSteeringAngle = map(steering, -127, 127, MAXRIGHT, MAXLEFT);
  
  frontLeft.write(steeringAngle);
  frontRight.write(steeringAngle);

  if (steeringMode == FOURWHEEL){
    rearRight.write(mirrorSteeringAngle);
    rearLeft.write(mirrorSteeringAngle);
  }else{
    rearRight.write(90);
    rearLeft.write(90);
  }

  //### DRIVE
  if (engageBrake){
    motorBrake();
  }else{
    // do nothing when both getting forward and backward
    if (speedForward > 0 && speedBackward > 0) {
      motorStop();
    }else {
       if (speedForward > 0){
          motorForward();
          setMotorSpeed(map(speedForward, 0, 255, 0, MAXSPEED));
       }else if (speedBackward > 0){
          motorBackward();
          setMotorSpeed(map(speedBackward, 0, 255, 0, MAXSPEED));
       }else {
        motorStop();
       }
    }
  }
  

  if (DIAGNOSTICS){
    printDiagnostics();
  }
  
}

void printDiagnostics() {

  char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)

  sprintf(buffer,"SPEED: %d, STEERING: %d, DEATHWHEEL: %d", speedForward, steering, deathWheel);  
  Serial.print(buffer);
  Serial.print(" STMODE: ");
  Serial.println(steeringMode);
}

void checkController() {
//  if (PS4.Right()) Serial.println("Right Button");
//  if (PS4.Down()) Serial.println("Down Button");
//  if (PS4.Up()) Serial.println("Up Button");
//  if (PS4.Left()) Serial.println("Left Button");
//
//  if (PS4.Square()) Serial.println("Square Button");
//  if (PS4.Cross()) Serial.println("Cross Button");
//  if (PS4.Circle()) Serial.println("Circle Button");
if (PS4.Triangle()) {
  if (!triangleDebounce){
    switch(deathWheel){
      case 0:
        deathWheel = 200;
        break;
      case 200:
        deathWheel = 0;
        break;  
    }
  }
  triangleDebounce = true;
}else{
  triangleDebounce = false;
}
//
//  if (PS4.UpRight()) Serial.println("Up Right");
//  if (PS4.DownRight()) Serial.println("Down Right");
//  if (PS4.UpLeft()) Serial.println("Up Left");
//  if (PS4.DownLeft()) Serial.println("Down Left");
//
//  if (PS4.L1()) Serial.println("L1 Button");
  if (PS4.R1()) {
    engageBrake = true;
  }else{
    engageBrake = false;
  }
//
//  if (PS4.Share()) Serial.println("Share Button");
  if (PS4.Options()){
    if (!optionDebounce){
      switch(steeringMode){
        case TWOWHEEL:
          steeringMode = FOURWHEEL;
          break;
        case FOURWHEEL:
          steeringMode = TWOWHEEL;
          break;
        default:
          break;    
      }
    }
    optionDebounce = true;
  }else{
    optionDebounce = false;
  }
//  if (PS4.L3()) Serial.println("L3 Button");
//  if (PS4.R3()) Serial.println("R3 Button");
//
//  if (PS4.PSButton()) Serial.println("PS Button");
//  if (PS4.Touchpad()) Serial.println("Touch Pad Button");
//
  if (PS4.L2()) {
    speedBackward = PS4.L2Value();
  }else{
    speedBackward = 0;
  }
  
  if (PS4.R2()) {
    speedForward = PS4.R2Value();
  }else{
    speedForward = 0;
  }

  if (PS4.LStickX()) {
    steering = PS4.LStickX();
  }else{
    steering = 0;
  }
//  if (PS4.LStickY()) {
//    Serial.printf("Left Stick y at %d\n", PS4.LStickY());
//  }
//  if (PS4.RStickX()) {
//    Serial.printf("Right Stick x at %d\n", PS4.RStickX());
//  }
//  if (PS4.RStickY()) {
//    Serial.printf("Right Stick y at %d\n", PS4.RStickY());
//  }

//  if (PS4.Charging()) Serial.println("The controller is charging");
//  if (PS4.Audio()) Serial.println("The controller has headphones attached");
//  if (PS4.Mic()) Serial.println("The controller has a mic attached");


}

void motorStop() {
  digitalWrite(MOTOR_ENABLE, LOW);
  Serial.println("STOP");
}

void motorBrake() {
  digitalWrite(MOTOR_ENABLE, LOW);
  digitalWrite(INB, LOW);
  digitalWrite(INA, LOW);
}

void setMotorSpeed(int motorSpeed) {
  analogWrite(MOTOR_ENABLE, motorSpeed);
  Serial.println(motorSpeed);
}

void motorBackward() {
  digitalWrite(INB, LOW);
  digitalWrite(INA, HIGH);
}

void motorForward() {
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
}
