/**
 * OrphyBot Test Bed
 * 
 * Receive navigation input thru HC-12 and translate into instructions for motors
 * 
 * Format of Input Received Thru HC-12:
 * [{direction}-{speed}-{duration}]
 * 
 * where:
 * {direction} - single digit; 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
 * {speed} - 3 digits (000 - 255)
 * {duration} - 4 digits (0000 - 9999) indicating milliseconds
 * 
 * example:
 * [1-099-2000] forward direction - speed 99 - 2 seconds
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

// constants
const char delimiterStart = '[';
const char delimiterEnd = ']';
const int msgLength = 12;
const String msgFormat = "[x-xxx-xxxx]";

// input
boolean txEnd = false;
byte incomingByte;
String readBuffer = "";
int dir = 0; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
int spd = 0; // 0 to 255
int durationMs = 0;

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

  Serial.begin(9600);
  delay(3000); // wait 3 seconds before starting 
  Serial.println("BEGIN!");
}
/************************END SETUP********************************/

/************************BEGIN LOOP*******************************/
void loop() {
  // capture transmission
  while (Serial.available()){
    incomingByte = Serial.read();
    readBuffer += char(incomingByte);
    
    if (incomingByte == delimiterEnd){
      txEnd = true;
    } 
//    else if (incomingByte == delimiterStart){
//      txEnd = false;
//      readBuffer = "";
//    }
  }

//txEnd = true;
//readBuffer = "[1-099-2000]";
  if (txEnd && readBuffer != ""){
//  if (txEnd && readBuffer != "" && readBuffer.length() == msgFormat.length()){
//    Serial.println(readBuffer);

    dir = readBuffer.substring(1,2).toInt();
    spd = readBuffer.substring(3,6).toInt();
    durationMs = readBuffer.substring(7,11).toInt();

    Serial.println("[" + String(dir) + "," + String(spd) + "," + String(durationMs) + "]");

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
        setFrontLeftMotor(spd, 2);
        setFrontRightMotor(spd, 1);
        setRearLeftMotor(spd, 2);
        setRearRightMotor(spd, 1);
        break;
      case 4: // right
        setFrontLeftMotor(spd, 1);
        setFrontRightMotor(spd, 2);
        setRearLeftMotor(spd, 1);
        setRearRightMotor(spd, 2);
        break;
      case 0: // stop
      default:
        setFrontLeftMotor(spd, 0);
        setFrontRightMotor(spd, 0);
        setRearLeftMotor(spd, 0);
        setRearRightMotor(spd, 0);
        break;
    }

    if (durationMs > 0){
      delay(durationMs);
    }

//delay(2000);
    readBuffer = "";
    txEnd = false;
    Serial.flush();
  } 

//  setFrontLeftMotor(spd, 0);
//  setFrontRightMotor(spd, 0);
//  setRearLeftMotor(spd, 0);
//  setRearRightMotor(spd, 0);

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
