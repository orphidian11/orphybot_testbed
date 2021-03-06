/**
 * OrphyBot Test Bed
 * Receiver Unit
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

#include<Wire.h>

// I2C followers address
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
const char DELIMITER_START = '['; // const unsigned long DELIMITER_START = 0b1100; 
const char DELIMITER_END = ']'; // const unsigned long DELIMITER_END = 0b011; 
const int MSG_LENGTH = 12;
//const String msgFormat = "[x-xxx-xxxx]";
const int COMMAND_LIST_SIZE = 10; // maximum of 10 commands can be sent
const unsigned long SENSOR_REQ_DELAY = 250; // delay between each request for sensor data
const float STOP_DISTANCE_MIN = 0.11;
const float STOP_DISTANCE_MAX = 0.15;
const float REVERSE_DISTANCE_MIN = 0.0;
const float REVERSE_DISTANCE_MAX = 0.1;

// input
boolean txEnd = false;
byte incomingByte;
String readBuffer = "";
int dir = 0; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
int spd = 0; // 0 to 255
int durationMs = 0;
unsigned long rxMsg = 0; // 32 bit message
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
int isSensorOverride = 0; // true or false

// sensor data structure
struct SensorData {
  float voltage;
  float distance;
};
SensorData sensorData;

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
  
  Wire.begin(); // leader mode

  Serial.begin(9600);
  Serial.println("RX BEGIN!");

  prevMillis = millis();
}
/************************END SETUP********************************/

/************************BEGIN LOOP*******************************/
void loop() {
  // capture transmission
  while (Serial.available()){
    incomingByte = Serial.read();
    readBuffer += char(incomingByte);
    
    if (incomingByte == DELIMITER_END){
      txEnd = true;
    } else if (incomingByte == DELIMITER_START) {
      txEnd = false;
    }
  }
    
  // request sensor information from sub-system
  currMillis = millis();
  if ((currMillis - prevMillis) > SENSOR_REQ_DELAY){
    prevMillis = currMillis;
    Wire.requestFrom(SENSORS_SUBSYS_ADDR, SENSORS_SUBSYS_ANSSIZE);
    Wire.readBytes((byte *)&sensorData, SENSORS_SUBSYS_ANSSIZE);
    Serial.println("V: " + String(sensorData.voltage) + " / D: " + String(sensorData.distance) + " / ms: " + prevMillis);
    isSensorOverride = 1;
    readBuffer = "000000000000"; // dummy value
  }

  // move the motors
  if ((txEnd && readBuffer != "") || isSensorOverride){
    int i;
    
    // split the commands contained in the readBuffer
    int numberOfCommands = readBuffer.length() / MSG_LENGTH;
    String commandList[COMMAND_LIST_SIZE];
    if (numberOfCommands <= 1){ 
      // if there is only one command sent...
      commandList[0] = readBuffer;
    } else { 
      // if there are multiple commands sent
      for (i = 0; i < numberOfCommands; i++){
        commandList[i] = readBuffer.substring(0 + (i * MSG_LENGTH), MSG_LENGTH + (i * MSG_LENGTH));
      }
    }

    // iterate through commandList
    for (i = 0; i < numberOfCommands; i++){
      if (isSensorOverride) {
        // overrides based on sensor data
        spd = 255;
        durationMs = 0;
        isSensorOverride = 0;
        
        if (sensorData.distance >= STOP_DISTANCE_MIN && sensorData.distance <= STOP_DISTANCE_MAX){
          dir = 0; // stop if STOP_DISTANCE is reached
        } else if (sensorData.distance <= REVERSE_DISTANCE_MAX){
          dir = 2; // reverse if REVERSE_DISTANCE is reached
        }
      } else {
        // tx signals
        rxMsg = atol(commandList[i].substring(1,12).c_str());
        dir = (rxMsg & 0b00001110000000000000000000000000) >> 25; 
        spd = (rxMsg & 0b00000001111111100000000000000000) >> 17; 
        durationMs = (rxMsg & 0b00000000000000011111111111111000) >> 3; 
      }
  
//      Serial.println("{" + String(dir) + "," + String(spd) + "," + String(durationMs) + "}");
  
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
  
      // if delay is provided, proceed then stop
      if (durationMs > 0){
        delay(durationMs);
        setFrontLeftMotor(spd, 0);
        setFrontRightMotor(spd, 0);
        setRearLeftMotor(spd, 0);
        setRearRightMotor(spd, 0);
      } 
    }
  
    readBuffer = "";
    txEnd = false;
  } 
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
