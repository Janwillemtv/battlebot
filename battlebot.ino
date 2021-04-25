#include <ESP32Servo.h>

#include <stdio.h>

#include "Wheel.h"
#include "ButtonMaker.h"
#include "EEPROM.h"

//## define the controller type here
#define PS4Controls
//#define PS3Controls

//### BLUETOOTH MAC ADDRESS
char * mac = "14:C2:13:14:9C:7D";

#ifdef PS3Controls
#include <Ps3Controller.h>
#endif

#ifdef PS4Controls
#include <PS4Controller.h>
#endif

#define MAX_OUT_CHARS 64  //max nbr of characters to be sent on diagnostic demand

#define R_PWM 32
#define L_PWM 33
#define M_ENABLE 25


//### CONFIG ######################
#define DIAGNOSTICS false
#define DIAGNOSTICSRATE 100
long diagnosticsTimer = 0;


//### SPEED SETTINGS
int MAXSPEED = 120;
enum spdMode {
  SLOW,
  NORMAL,
  INSANE
};

//### STEERING MODES
#define TWOWHEEL 0
#define RACE 1
#define FOURWHEEL 2


//object wrappers that turn the PS4 continous output into buttons
ButtonMaker up, down, left, right, options, triangle, L3;

//### INIT ########################
int steeringMode = TWOWHEEL;
int crabSteering = 0;

//### adjust steering efficacy based on speed
bool speedAdjust = false;

spdMode speedMode = SLOW;

float predictedSpeed = 0;
long speedUpdateTimer = 0;

int speedForward = 0;
int speedBackward = 0;
int steering = 0;
int deathWheel = 0;

bool engageBrake = false;

// 16 servo objects can be created on the ESP32
//### WHEELS, first entry is timer and config, second is servo pin
Wheel FL(0, 13);
Wheel FR(1, 14);
Wheel RL(2, 12);
Wheel RR(3, 27);

// LEFT and RIGHT arrow on PS4 pad change the steering offset
// OFFSET is stored in EEPROM
int steeringOffset = 0;
const int OFFSETADDRESS = 0;

// Maybe create a driving mode where the max range changes variably with the speed?
// And a race mode perhaps? with 50% steering transfer for the rear wheels and 100% for the front wheels?
const int maxRange = 50;

// Recommended PWM GPIO pins on the ESP32 include 2,4,12-19,21-23,25-27,32-33

void safeOffset() {
  EEPROM.writeInt(OFFSETADDRESS, steeringOffset);
  EEPROM.commit();
}

int getOffset() {
  return EEPROM.readInt(OFFSETADDRESS);
}

void setup() {
  Serial.begin(115200);
#ifdef PS4Controls
  PS4.begin(mac);
#endif
#ifdef PS3Controls
  Ps3.begin(mac);
#endif

  Serial.println("Ready.");

  //Serial.printf("Battery Level : %d\n", PS4.Battery());

  steeringOffset = getOffset();
  Serial.print("offset = ");
  Serial.println(steeringOffset);
  // ##### STEERING
  //Set zero position of the servos -> manual entry
  FL.setZero(90);
  FR.setZero(93);
  RL.setZero(96);
  RR.setZero(90);

  //apply offset to wheels
  setOffset(steeringOffset);

  // set max range -> this could be used to change efficacy of steering for race mode
  // or speed based steering
  staticSteering();

  //### DRIVE
  pinMode(M_ENABLE, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

}

//### LOOP #########################
void loop() {
  // check connection to controller
  bool controllerConnected = false;
#ifdef PS4Controls
  controllerConnected = PS4.isConnected();
  if (controllerConnected) checkPS4Controller();
#endif
#ifdef PS3Controls
  controllerConnected = Ps3.isConnected();
  if (controllerConnected) checkPS3Controller();
#endif
  if (!controllerConnected) disableBot();

  //### DRIVE
  MAXSPEED = getMaxSpeed();
  int spd = 0; // storage variable for variable steering mode or whatever

  if (engageBrake) motorBrake(); // magnetic braking
  else
  {
    enableMotor();
    // do nothing when both getting forward and backward
    if (speedForward > 0 && speedBackward > 0) disableMotor();
    else
    {
      if (speedForward > 0)
      {
        spd = map(speedForward, 0, 255, 0, MAXSPEED);
        motorForward(spd);
      }
      else if (speedBackward > 0)
      {
        spd = -map(speedBackward, 0, 255, 0, MAXSPEED);
        motorBackward(-spd);
      }
      else disableMotor();
    }
  }
  //### predict the current velocity
  predictSpeed(spd);

  //## apply predicted speed on steering efficacy
  if (speedAdjust) adjustSteering(spd);
  else staticSteering();

  //### STEERING
  if (engageBrake) steerPizza(); // mechanical braking by making a triangle
  else
  {
    int combiFront = steering + crabSteering;
    if (combiFront > 127) combiFront = 127;
    else if (combiFront < -127) combiFront = -127;

    if (steeringMode == TWOWHEEL)
    {
      FL.steer(combiFront);
      FR.steer(combiFront);
      RL.steer(-crabSteering);
      RR.steer(-crabSteering);
    }
    else if (steeringMode == RACE)
    {
      int combiBack = int(steering / 2) - crabSteering;
      if (combiBack > 127) combiBack = 127;
      else if (combiBack < -127) combiBack = -127;

      FL.steer(combiFront);
      FR.steer(combiFront);
      RL.steer(combiBack);
      RR.steer(combiBack);
    }
    else if (steeringMode == FOURWHEEL)
    {
      int combiBack = steering - crabSteering;
      if (combiBack > 127) combiBack = 127;
      else if (combiBack < -127) combiBack = -127;

      FL.steer(combiFront);
      FR.steer(combiFront);
      RL.steer(combiBack);
      RR.steer(combiBack);
    }
  }

  if (DIAGNOSTICS) printDiagnostics();
}

bool isClose(float value, float comparison)
{
  return abs(comparison - value) < 1;
}

// predicts the actual speed
void predictSpeed(int spd)
{
  if (millis() > speedUpdateTimer + 20)
  {
    if (isClose(predictedSpeed, 0) && spd == 0) predictedSpeed = 0;
    else
    {
      float added = (float(spd) - predictedSpeed);
      predictedSpeed += (added * (engageBrake ? 0.07 : 0.02));
    }
    speedUpdateTimer = millis();
  }
}

//### PIZZA
// This sets both wheels into a pizza triangle into the direction of movement.
// They jam towards the direction of movement to not cause tensile stress but rather
// compressive stress, which is much less harmful
void steerPizza()
{
  //symmetric pizza
  if (predictedSpeed == 0)
  {
    FL.steer(127);
    FR.steer(-127);
    RL.steer(127);
    RR.steer(-127);
  }
  //forwards pizza
  else if (predictedSpeed > 0)
  {
    FL.steer(127);
    FR.steer(-127);
    RL.steer(-127);
    RR.steer(127);
  }
  //backwards pizza
  else
  {
    FL.steer(-127);
    FR.steer(127);
    RL.steer(127);
    RR.steer(-127);
  }
}

bool printDiagnostics() {

  if (millis() > diagnosticsTimer + DIAGNOSTICSRATE)
  {
    //char buffer[MAX_OUT_CHARS + 1];  //buffer used to format a line (+1 is for trailing 0)
    //sprintf(buffer, "SPEED: %d, STEERING: %d, DEATHWHEEL: %d", predictedSpeed, steering, deathWheel);
    //Serial.print(buffer);
    Serial.print("SPEED: ");
    Serial.println(predictedSpeed);
    Serial.print(" STMODE: ");
    Serial.println(steeringMode);
    diagnosticsTimer = millis();
    return true;
  }
  return false;
}

int getMaxSpeed() {
  if (speedMode == SLOW) return 50;
  else if (speedMode == NORMAL) return 120;
  else if (speedMode == INSANE) return 255;
  else return 50;
}

void checkPS4Controller() {
#ifdef PS4Controls
  if (down.isPress(PS4.Down()))
  {
    if (speedMode == NORMAL) speedMode = SLOW;
    else if (speedMode == INSANE) speedMode = NORMAL;
  }
  if (up.isPress(PS4.Up()))
  {
    if (speedMode == SLOW) speedMode = NORMAL;
    else if (speedMode == NORMAL) speedMode = INSANE;
  }
  if (left.isPress(PS4.Left())) {
    steeringOffset --;
    setOffset(steeringOffset);
    safeOffset();
  }
  if (right.isPress(PS4.Right()))
  {
    steeringOffset ++;
    setOffset(steeringOffset);
    safeOffset();
  }

  if (triangle.isPress(PS4.Triangle()))
  {
    switch (deathWheel) {
      case 0:
        deathWheel = 200;
        break;
      case 200:
        deathWheel = 0;
        break;
    }
  }

  if (options.isPress(PS4.Options()))
  {
    steeringMode ++;
    if (steeringMode > FOURWHEEL) steeringMode = TWOWHEEL;
  }
  if (L3.isPress(PS4.L3())) speedAdjust = !speedAdjust;

  engageBrake = (PS4.R1() || PS4.Cross());

  if (PS4.L2()) speedBackward = PS4.L2Value();
  else speedBackward = 0;

  if (PS4.R2())speedForward = PS4.R2Value();
  else speedForward = 0;

  if (PS4.LStickX()) steering = PS4.LStickX();
  else steering = 0;

  if (PS4.RStickX()) crabSteering = PS4.RStickX();
  else crabSteering = 0;
#endif
}

void checkPS3Controller() {
#ifdef PS3Controls
  if (Ps3.event.button_down.down)
  {
    if (speedMode == NORMAL) speedMode = SLOW;
    else if (speedMode == INSANE) speedMode = NORMAL;
  }
  if (Ps3.event.button_down.up)
  {
    if (speedMode == SLOW) speedMode = NORMAL;
    else if (speedMode == NORMAL) speedMode = INSANE;
  }
  if (Ps3.event.button_down.left) {
    steeringOffset --;
    setOffset(steeringOffset);
    safeOffset();
  }
  if (Ps3.event.button_down.right)
  {
    steeringOffset ++;
    setOffset(steeringOffset);
    safeOffset();
  }

  if (Ps3.event.button_down.triangle)
  {
    switch (deathWheel) {
      case 0:
        deathWheel = 200;
        break;
      case 200:
        deathWheel = 0;
        break;
    }
  }

  if (Ps3.event.button_down.select)
  {
    steeringMode ++;
    if (steeringMode > FOURWHEEL) steeringMode = TWOWHEEL;
  }
  if (Ps3.event.button_down.l3) speedAdjust = !speedAdjust;

  engageBrake = (Ps3.data.button.cross || Ps3.data.button.r1);

  speedForward = Ps3.data.analog.button.r2;
  speedBackward = Ps3.data.analog.button.l2;

  steering = Ps3.data.analog.stick.lx;
  crabSteering = Ps3.data.analog.stick.rx;
#endif
}

void staticSteering()
{
  FL.setMaxRange(maxRange);
  FR.setMaxRange(maxRange);
  RL.setMaxRange(maxRange);
  RR.setMaxRange(maxRange);
}

void adjustSteering(int spd)
{
  int absSPD = abs(spd);
  int cut = 120;
  if (absSPD < 50) absSPD = int(absSPD / 2);
  else if (absSPD > 120) absSPD = cut + int((absSPD - cut) / 10);
  int range = map(absSPD, 0, 135, maxRange, 10);
  FL.setMaxRange(range);
  FR.setMaxRange(range);
  RL.setMaxRange(range);
  RR.setMaxRange(range);
}

void setOffset(int offset)
{
  FL.setOffset(offset);
  FR.setOffset(offset);
  RL.setOffset(offset);
  RR.setOffset(offset);
}

void disableBot()
{
  speedForward = 0;
  speedBackward = 0;
  steering = 0;
  crabSteering = 0;
  deathWheel = 0;
  disableMotor();
}

void enableMotor() {
  digitalWrite(M_ENABLE, HIGH);
  //Serial.println("motor ON");
}

void disableMotor() {
  digitalWrite(M_ENABLE, LOW);
  digitalWrite(L_PWM, LOW);
  digitalWrite(R_PWM, LOW);
  //Serial.println("motor STOP");
}

void motorBackward(int spd) {
  digitalWrite(L_PWM, LOW);
  analogWrite(R_PWM, spd);
}

void motorForward(int spd) {
  digitalWrite(R_PWM, LOW);
  analogWrite(L_PWM, spd);
}
void motorBrake() {
  digitalWrite(M_ENABLE, HIGH);
  digitalWrite(L_PWM, LOW);
  digitalWrite(R_PWM, LOW);
}
