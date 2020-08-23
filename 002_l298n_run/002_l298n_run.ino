/**
 * OrphyBot Test Bed
 * 
 * Run the DC motors using the L298N drivers
 */

// Front Left Motor
#define F_L_SPD 3 // pwm
#define F_L_A 2
#define F_L_B 4

// Front Right Motor
#define F_R_SPD 10 // pwm
#define F_R_A 6
#define F_R_B 5

// Rear Left Motor
#define R_L_SPD 9 // pwm
#define R_L_A 8
#define R_L_B 7

// Rear Right Motor
#define R_R_SPD 11 // pwm
#define R_R_A 12
#define R_R_B 13

/************************BEGIN SETUP******************************/
void setup() {
  // put your setup code here, to run once:
  pinMode(F_L_SPD, OUTPUT);
  pinMode(F_L_A, OUTPUT);
  pinMode(F_L_B, OUTPUT);
  pinMode(F_R_SPD, OUTPUT);
  pinMode(F_R_A, OUTPUT);
  pinMode(F_R_B, OUTPUT);
  pinMode(R_L_SPD, OUTPUT);
  pinMode(R_L_A, OUTPUT);
  pinMode(R_L_B, OUTPUT);
  pinMode(R_R_SPD, OUTPUT);
  pinMode(R_R_A, OUTPUT);
  pinMode(R_R_B, OUTPUT);
  
  // wait for 5 seconds before running
//  delay(5000);
}
/************************END SETUP********************************/

/************************BEGIN LOOP*******************************/
void loop() {
  // wait for 2 seconds before starting
  delay(2000);

  int spd = 255;
  
  // put your main code here, to run repeatedly:
  // move forward for 2 seconds
  setFrontLeftMotor(spd, 1);
  setFrontRightMotor(spd, 1);
  setRearLeftMotor(spd, 1);
  setRearRightMotor(spd, 1);

  delay(2000);

  // stop
  setFrontLeftMotor(spd, 0);
  setFrontRightMotor(spd, 0);
  setRearLeftMotor(spd, 0);
  setRearRightMotor(spd, 0);

  delay(2000);

  // move backward for 2 seconds
  setFrontLeftMotor(spd, 2);
  setFrontRightMotor(spd, 2);
  setRearLeftMotor(spd, 2);
  setRearRightMotor(spd, 2);

  delay(2000);

  // stop
  setFrontLeftMotor(spd, 0);
  setFrontRightMotor(spd, 0);
  setRearLeftMotor(spd, 0);
  setRearRightMotor(spd, 0);

  delay(2000);

  // spin left for 2 seconds
  setFrontLeftMotor(spd, 2);
  setFrontRightMotor(spd, 1);
  setRearLeftMotor(spd, 2);
  setRearRightMotor(spd, 1);

  delay(2000);

  // stop
  setFrontLeftMotor(spd, 0);
  setFrontRightMotor(spd, 0);
  setRearLeftMotor(spd, 0);
  setRearRightMotor(spd, 0);

  delay(2000);

  // spin right for 2 seconds
  setFrontLeftMotor(spd, 1);
  setFrontRightMotor(spd, 2);
  setRearLeftMotor(spd, 1);
  setRearRightMotor(spd, 2);

  delay(2000);

  // stop
  setFrontLeftMotor(spd, 0);
  setFrontRightMotor(spd, 0);
  setRearLeftMotor(spd, 0);
  setRearRightMotor(spd, 0);

  delay(2000);

}
/************************END LOOP*********************************/

/************************BEGIN PRIVATE FUNCTIONS******************/

/**
 * Set motion of front left motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setFrontLeftMotor(int spd, int dir){
  setMotor(F_L_SPD, F_L_A, F_L_B, spd, dir);
}

/**
 * Set motion of front right motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setFrontRightMotor(int spd, int dir){
  setMotor(F_R_SPD, F_R_A, F_R_B, spd, dir);
}

/**
 * Set motion of rear left motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setRearLeftMotor(int spd, int dir){
  setMotor(R_L_SPD, R_L_A, R_L_B, spd, dir);
}

/**
 * Set motion of rear right motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setRearRightMotor(int spd, int dir){
  setMotor(R_R_SPD, R_R_A, R_R_B, spd, dir);
}

/**
 * General function for setting motion of all motors
 * @param spdPin speed pin
 * @param aPin A pin
 * @param bPin B pin
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setMotor(int spdPin, int aPin, int bPin, int spd, int dir){
  analogWrite(spdPin, spd);
  
  if (dir == 1){
    digitalWrite(aPin, HIGH);
    digitalWrite(bPin, LOW);
  } else if (dir == 2) {
    digitalWrite(aPin, LOW);
    digitalWrite(bPin, HIGH);
  } else {
    digitalWrite(aPin, LOW);
    digitalWrite(bPin, LOW);
  }
}

/************************END PRIVATE FUNCTIONS********************/
