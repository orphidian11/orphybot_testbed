/**
 * Orphybot Arm
 */

#include <Wire.h>
#include <Stepper.h>

// I2C followers address
#define I2C_CTRL_ADD 11
#define I2C_ARM_ADD 4 

// stepper 1
#define pin1IN1 5
#define pin1IN2 4
#define pin1IN3 3
#define pin1IN4 2

// stepper 2
#define pin2IN1 9
#define pin2IN2 8
#define pin2IN3 7
#define pin2IN4 6

// stepper 3
#define pin3IN1 13
#define pin3IN2 12
#define pin3IN3 11
#define pin3IN4 10

const int STEPS = 2;
const int STEPS_PER_REVO = 2048; // for ROHS 28BYJ-48 5V DC stepper motor
const int STEP_SPEED = 10; // RPM
 const int ROT_CW = 1; // clockwise
 const int ROT_CCW = -1; // counter clockwise

struct Coords {
  int x;
  int y;
  int z;
  int deg; // 0 - 360 degrees
  bool lBtn;
  bool rBtn;
};

Coords currCoords = Coords(); // current local value of coords
Coords newCoords = Coords();

// Stepper class constructor format
// Note that 3rd parameter is switched, which is not that intuitive
// Stepper myStepper1(STEPS_PER_REVO, pin1IN1, pin1IN3, pin1IN2, pin1IN4);
// Stepper myStepper2(STEPS_PER_REVO, pin2IN1, pin2IN3, pin2IN2, pin2IN4);
 Stepper myStepper3(STEPS_PER_REVO, pin3IN1, pin3IN3, pin3IN2, pin3IN4);

void setup() {
  Wire.begin(I2C_ARM_ADD); // follower mode
  Wire.onReceive(receiveEvent); 

  // myStepper1.setSpeed(STEP_SPEED);
  // myStepper2.setSpeed(STEP_SPEED);
   myStepper3.setSpeed(STEP_SPEED);

  Serial.begin(9600);
  Serial.println("*** Orphybot Arm starting... ***");
}

void loop() {
   Serial.print("[" + 
     String(currCoords.x) + "," + 
     String(currCoords.y) + "," + 
     String(currCoords.deg) + "," +
     String(currCoords.lBtn) + "," +
     String(currCoords.rBtn) + "]");
     
   moveArm();

   updateCoords();

   Serial.println("");
}

void updateCoords(){
  if (newCoords.x != currCoords.x ||
    newCoords.y != currCoords.y ||
    newCoords.deg != currCoords.deg ||
    newCoords.lBtn != currCoords.lBtn ||
    newCoords.rBtn != currCoords.rBtn){
    currCoords = newCoords;
  }
}

void receiveEvent(int howMany){
  Wire.readBytes((byte *)&newCoords, howMany);
}

void moveArm(){
  // rotate base
  if (newCoords.deg != currCoords.deg){ 
    int rotDir = 0;
    if (newCoords.deg > currCoords.deg){
      rotDir = ROT_CW;
      Serial.print("RIGHT");
    } else {
      rotDir = ROT_CCW;
      Serial.print("LEFT");
    }

    myStepper3.step(STEPS * rotDir);
  }

    myStepper3.step(STEPS * ROT_CW);
//  Serial.print("] ");

  currCoords = newCoords;
//  delay(100);
}
