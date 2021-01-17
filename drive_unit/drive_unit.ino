/**
 * OrphyBot Test Bed
 * Drive Unit (Arduino UNO)
 * In charge of running the motors. As follower, receives drive instructions from Sensor Unit via I2C
 * 
 * Connected Devices:
 * - (D7-D9, D11-D13) 1x L298N motor drivers with 2x 9v DC motors each 
 * - (A4:SDA, A5:SCL) Arduino NANO as leader unit via I2C 
 */

#include<Wire.h>
#include<QueueList.h>

// I2C addresses
#define DRIVE_SUBSYS_ADDR 8
#define SENSORS_SUBSYS_ADDR 9
#define SENSORS_SUBSYS_ANSSIZE 8 // character length

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

// constants
const uint8_t DIR_MIN = 0;
const uint8_t DIR_MAX = 4;
const uint8_t SPD_MIN = 0;
const uint8_t SPD_MAX = 255;
const uint8_t DURATION_MS_MIN = 0;
const uint8_t DURATION_MS_MAX = 9999;
const int DRIVE_QUEUE_LIMIT = 1; 

// drive data structure 
struct DriveData {
  uint8_t spd;
};

// drive command structure
struct DriveCommand {
  uint8_t dir; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
  uint8_t spd; // 0 to 255
  uint8_t durationMs; // in milliseconds
};

QueueList<DriveCommand> driveQueue;

/*********
 * SETUP *
 *********/

void setup() {
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
  
  Wire.begin(DRIVE_SUBSYS_ADDR); // follower mode
  Wire.onReceive(receiveDriveCommand); 
  Wire.onRequest(sendSpeedData);

  Serial.begin(9600);
  Serial.println("DRIVE UNIT BEGIN!");
}

/**
 * Receive the drive command and add it to the queue
 */
void receiveDriveCommand(int numBytes){
//  Serial.println("receiveDriveCommand");
  DriveCommand driveCommand;
  Wire.readBytes((byte *)&driveCommand, numBytes);
//  Serial.println("[" + String(driveQueue.count()) + "] / dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
  if (driveQueue.count() < DRIVE_QUEUE_LIMIT){
    driveQueue.push(driveCommand);
  }
}

/**
 * Send back speed data
 */
void sendSpeedData(){
//  Serial.println("sendSpeedData");
  DriveData driveData;
  driveData.spd = 100; // dummy data for now
  Serial.println("SEND >> driveData: " + String(driveData.spd) + " / sizeof " + String(sizeof(driveData)));
  Wire.write((byte *)&driveData, sizeof driveData);
}

/********
 * LOOP *
 ********/
void loop() {
  // check the drive queue
  while(!driveQueue.isEmpty()){
    DriveCommand driveCommand = driveQueue.peek();
    Serial.println("**[" + String(driveQueue.count()) + "] / dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
    runDriveCommand(driveCommand);
    driveQueue.pop();
  }
}

/*********************
 * PRIVATE FUNCTIONS *
 *********************/

void runDriveCommand(DriveCommand driveCommand){
  // verify that all the values are valid 
  if (driveCommand.dir < DIR_MIN || driveCommand.dir > DIR_MAX 
    || driveCommand.spd < SPD_MIN || driveCommand.spd > SPD_MAX
    || driveCommand.durationMs < DURATION_MS_MIN || driveCommand.durationMs > DURATION_MS_MAX){
    return; 
  }
  
  uint8_t dir = driveCommand.dir;
  uint8_t spd = driveCommand.spd;
  uint8_t durationMs = driveCommand.durationMs;

  Serial.println("RUN << dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
  
  switch(dir){
    case 1: // forward
      setFrontLeftMotor(spd, 1);
      setFrontRightMotor(spd, 1);
      setRearLeftMotor(spd, 1);
      setRearRightMotor(spd, 1);
      break;
    case 2: // reverse
      setFrontLeftMotor(spd, 2);
      setFrontRightMotor(spd, 2);
      setRearLeftMotor(spd, 2);
      setRearRightMotor(spd, 2);
      break;
    case 3: // left
      setFrontLeftMotor(spd, 1);
      setFrontRightMotor(spd, 2);
      setRearLeftMotor(spd, 1);
      setRearRightMotor(spd, 2);
      break;
    case 4: // right
      setFrontLeftMotor(spd, 2);
      setFrontRightMotor(spd, 1);
      setRearLeftMotor(spd, 2);
      setRearRightMotor(spd, 1);
      break;
    case 0: // stop
    default:
      setFrontLeftMotor(spd, 0);
      setFrontRightMotor(spd, 0);
      setRearLeftMotor(spd, 0);
      setRearRightMotor(spd, 0);
      break;
  }
  
  // if delay is provided, proceed then stop
  if (durationMs > 0){
    delay(durationMs);
    setFrontLeftMotor(spd, 0);
    setFrontRightMotor(spd, 0);
    setRearLeftMotor(spd, 0);
    setRearRightMotor(spd, 0);
  } 
  
}

/**
 * Set motion of front left motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setFrontLeftMotor(uint8_t spd, uint8_t dir){
  setMotor(F_L_SPD, F_L_A, F_L_B, spd, dir);
}

/**
 * Set motion of front right motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setFrontRightMotor(uint8_t spd, uint8_t dir){
  setMotor(F_R_SPD, F_R_A, F_R_B, spd, dir);
}

/**
 * Set motion of rear left motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setRearLeftMotor(uint8_t spd, uint8_t dir){
  setMotor(R_L_SPD, R_L_A, R_L_B, spd, dir);
}

/**
 * Set motion of rear right motor
 * @param spd speed of the motor [0-255]
 * @param dir direction [0: stop, 1: forward, 2: backward]
 */
void setRearRightMotor(uint8_t spd, uint8_t dir){
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
void setMotor(uint8_t spdPin, uint8_t aPin, uint8_t bPin, uint8_t spd, uint8_t dir){
  analogWrite(spdPin, spd);
  
  if (dir == 1){
    digitalWrite(bPin, HIGH);
    digitalWrite(aPin, LOW);
  } else if (dir == 2) {
    digitalWrite(bPin, LOW);
    digitalWrite(aPin, HIGH);
  } else {
    digitalWrite(aPin, LOW);
    digitalWrite(bPin, LOW);
  }
}
