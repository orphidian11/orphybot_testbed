/**
 * OrphyBot Test Bed
 * Drive Unit (Arduino UNO)
 * In charge of running the motors. As follower, receives drive instructions from Sensor Unit via I2C
 * 
 * Connected Devices:
 * - (D7-D9, D11-D13) 1x L298N motor drivers with 2x 9v DC motors each 
 * - (A4:SDA, A5:SCL) Arduino NANO as leader unit via I2C 
 * 
 * NOTE:
 * Code is based on XX_l298n_driver_03 sketch
 */

#include<Wire.h>
#include<QueueList.h>

// I2C addresses
#define DRIVE_SUBSYS_ADDR 8
#define SENSORS_SUBSYS_ADDR 9
#define SENSORS_SUBSYS_ANSSIZE 8 // character length

// Front Left Motor
#define F_L_SPD 10 // pwm
#define F_L_A 2
#define F_L_B 4

// Front Right Motor
#define F_R_SPD 3 // pwm
#define F_R_A 6
#define F_R_B 5

// Rear Left Motor
#define R_L_SPD 11 // pwm
#define R_L_A 8
#define R_L_B 7

// Rear Right Motor
#define R_R_SPD 9 // pwm
#define R_R_A 12
#define R_R_B 13

/** CONSTANTS **/
// motor speed ranges
const int SPD_STOP = 0;
const int SPD_MIN = 100;
const int SPD_MAX = 255;
const int DURATION_MS_MIN = 0;
const int DURATION_MS_MAX = 9999;
const int DRIVE_QUEUE_LIMIT = 1; 

// joystick ranges
const float J_MIN = 0;
const float J_MAX = 1023;

// joystick neutral range tolerance 
const int J_NEUT_MIN = 498; // 500
const int J_NEUT_MAX = 503; // 500

// drive data structure 
struct DriveData {
  int spd;
};

// strcture for commands to be transmitted
struct DriveCommand {
  int x; // joystick x-axis
  int y; // joystick y-axis
  int sw; // joystick switch
  int spd; // speed
  int durationMs; // duration in milliseconds
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
//  Serial.print("[" + String(driveQueue.count()) + "]);
//  if (driveQueue.count() < DRIVE_QUEUE_LIMIT){
//    driveQueue.push(driveCommand);
//    Serial.print("RECV << "); 
//    Serial.print("x: " + String(driveCommand.x) + " / ");
//    Serial.print("y: " + String(driveCommand.y) + " / ");
//    Serial.print("sw: " + String(driveCommand.sw) + " / ");
//    Serial.print("spd: " + String(driveCommand.spd) + " / "); 
//    Serial.println("durationMs: " + String(driveCommand.durationMs));
    
    runDriveCommand(driveCommand);
//  }
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
//  while(!driveQueue.isEmpty()){
//    DriveCommand driveCommand = driveQueue.peek();
//    Serial.println("**[" + String(driveQueue.count()) + "] / dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
//    runDriveCommand(driveCommand);
//    driveQueue.pop();
//  }
//  Serial.println("1");
}

/*********************
 * PRIVATE FUNCTIONS *
 *********************/

void runDriveCommand(DriveCommand driveCommand) {
  int lSpd;
  int rSpd;
  int lSpdRedux;
  int rSpdRedux;

  /** INPUT **/
  int xVal = driveCommand.x;
  int yVal = driveCommand.y;

  Serial.print("EXEC >> ");

  /** PROCESS **/
  // forward/reverse/stop
  if (yVal <= J_NEUT_MIN){
    // forward
    lSpd = rSpd = map(yVal, J_NEUT_MIN, J_MIN, SPD_MIN, SPD_MAX);
    setForward();
  } else if (yVal >= J_NEUT_MAX) {
    // reverse
    lSpd = rSpd = map(yVal, J_NEUT_MAX, J_MAX, SPD_MIN, SPD_MAX);
    setReverse();
//  } else {
//    lSpd = rSpd = SPD_STOP;
//    motorStop();
  }

  // left/right turn
  if (xVal <= J_NEUT_MIN){
    // left turn
    lSpdRedux = map(xVal, J_NEUT_MIN, J_MIN, SPD_MIN, lSpd);
    rSpdRedux = 0;
  } else if (xVal >= J_NEUT_MAX) {
    // right turn
    lSpdRedux = 0;
    rSpdRedux = map(xVal, J_NEUT_MAX, J_MAX, SPD_MIN, rSpd);
  } else {
    lSpdRedux = rSpdRedux = SPD_STOP;
  }
  
  // if neither forward nor reverse, but was set to left or right, do pivot turn
  if ((yVal > J_NEUT_MIN) &&  (yVal < J_NEUT_MAX)){ // yVal is within neutral zone
    if (xVal <= J_NEUT_MIN){
      // pivot left
      lSpdRedux = rSpdRedux = SPD_STOP;
      lSpd = rSpd = map(xVal, J_NEUT_MIN, J_MIN, SPD_MIN, SPD_MAX);
      setLeftReverse();
      setRightForward();
    } else if (xVal >= J_NEUT_MAX) {
      // pivot right
      lSpdRedux = rSpdRedux = SPD_STOP;
      lSpd = rSpd = map(xVal, J_NEUT_MAX, J_MAX, SPD_MIN, SPD_MAX);
      setRightReverse();
      setLeftForward();
    } else {
      // full stop
      lSpd = rSpd = SPD_STOP;
      lSpdRedux = rSpdRedux = SPD_STOP;
      motorStop();
    }
  } 

  // to prevent negative speed reduction, set reduction to 0 when speed is 0
  if (lSpd == SPD_MIN){
    lSpdRedux = SPD_STOP;
  } 
  if (rSpd == SPD_MIN){
    rSpdRedux = SPD_STOP;
  }

  /** OUTPUT **/
  analogWrite(F_L_SPD, lSpd - lSpdRedux);
  analogWrite(R_L_SPD, lSpd - lSpdRedux);
//  Serial.print("LSPD: " + String(lSpd - lSpdRedux) + " / ");
  Serial.print("LSPD: " + String(lSpd) + " - " + String(lSpdRedux) + " = " + String(lSpd - lSpdRedux) + " / ");
  
  analogWrite(F_R_SPD, rSpd - rSpdRedux);
  analogWrite(R_R_SPD, rSpd - rSpdRedux);
//  Serial.println("RSPD: " + String(rSpd - rSpdRedux) + " / ");
  Serial.println("RSPD: " + String(rSpd) + " - " + String(rSpdRedux) + " = " + String(rSpd - rSpdRedux) + " / ");
}

/**
 * Both wheels stop
 */
void motorStop(){
  Serial.print("STOP / ");

  // left
  digitalWrite(F_L_A, LOW);
  digitalWrite(F_L_B, LOW);
  
  digitalWrite(R_L_A, LOW);
  digitalWrite(R_L_B, LOW);

  // right
  digitalWrite(F_R_A, LOW);
  digitalWrite(F_R_B, LOW);
  
  digitalWrite(R_R_A, LOW);
  digitalWrite(R_R_B, LOW);
}

void setForward(){
  setLeftForward();
  setRightForward();
}

void setLeftForward(){
  Serial.print("LFWD / ");

  // left
  digitalWrite(F_R_A, LOW);
  digitalWrite(F_R_B, HIGH);
  
  digitalWrite(R_R_A, LOW);
  digitalWrite(R_R_B, HIGH);
}

void setRightForward(){
  Serial.print("RFWD / ");

  // right
  digitalWrite(F_L_A, LOW);
  digitalWrite(F_L_B, HIGH);
  
  digitalWrite(R_L_A, LOW);
  digitalWrite(R_L_B, HIGH);
}

void setReverse(){
  setLeftReverse();
  setRightReverse();
}

void setLeftReverse(){
  Serial.print("LREV / ");
  
  // left
  digitalWrite(F_R_A, HIGH);
  digitalWrite(F_R_B, LOW);
  
  digitalWrite(R_R_A, HIGH);
  digitalWrite(R_R_B, LOW);
}

void setRightReverse(){
  Serial.print("RREV / ");
  
  // right
  digitalWrite(F_L_A, HIGH);
  digitalWrite(F_L_B, LOW);
  
  digitalWrite(R_L_A, HIGH);
  digitalWrite(R_L_B, LOW);
}
