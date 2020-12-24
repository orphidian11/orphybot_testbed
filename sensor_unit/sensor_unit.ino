/**
 * OrphyBot Test Bed
 * Sensor Unit (Arduino NANO)
 * Capture signals from sensors and send to Drive Unit via I2C
 * 
 * Connected Devices:
 * - (A4:SDA, A5:SCL) Arduino UNO as follower unit via I2C 
 * - (A3) Voltage Sensor 
 * - (D2:Trig, D3:Echo) HC-SR04 Ultrasonic Distance Sensor
 * - (D0:RX, D1:TX) HC-12 transceiver
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
#include <SoftwareSerial.h>

// I2C followers addresses
#define DRIVE_SUBSYS_ADDR 8
#define DRIVE_DATA_ANSSIZE 2
#define SENSORS_SUBSYS_ADDR 9
#define SENSORS_SUBSYS_ANSSIZE 8 // number of characters
#define V_SENSOR A3
#define HCSR04_TRIG 2
#define HCSR04_ECHO 3
#define LED_PIN 13 
#define TX 10
#define RX 11

// constants
const float VIN_MIN = 0;
const float VIN_MAX = 1023;
const float VOUT_MIN = 0;
const float VOUT_MAX = 25;
const int MPS = 343; // speed of sound (m/s)
const int HC_SR04_PING_US = 10; // microsecond delay for HC-SR04 ping
const int HC_SR04_WAIT_US = 25; // microsecond delay for HC-SR04 delay
const char DELIMITER_START = '['; // const unsigned long DELIMITER_START = 0b1100; 
const char DELIMITER_END = ']'; // const unsigned long DELIMITER_END = 0b011; 
const int MSG_LENGTH = 12;
const int COMMAND_LIST_SIZE = 10; // maximum of 10 commands can be sent
const unsigned long SENSOR_REQ_DELAY = 250; // delay between each request for sensor data
const unsigned long DRIVE_POLL_MS = 500;
const float STOP_DISTANCE_MIN = 0.11;
const float STOP_DISTANCE_MAX = 0.15;
const float REVERSE_DISTANCE_MIN = 0.0;
const float REVERSE_DISTANCE_MAX = 0.1;

// input 
boolean txEnd = false;
byte incomingByte;
String readBuffer = "";
unsigned long prevMillis = 0;
unsigned long currMillis = 0;

// output 
float distance;
float vOut;

// sensor data structure
struct SensorData {
  float voltage; 
  float distance; // in meters
};

// drive data structure 
struct DriveData {
  int spd;
};

// drive command structure
struct DriveCommand {
  int dir; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
  int spd; // 0 to 255
  int durationMs; // in milliseconds
};

// telemetry structure 
struct Telemetry {
  DriveData drive;
  SensorData sensor;
};

SoftwareSerial hc12(TX,RX);

/*********
 * SETUP *
 *********/
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(V_SENSOR, INPUT);
  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);
  
  Wire.begin(); // run in leader mode
  hc12.begin(9600);

  Serial.begin(9600);
  Serial.println("SENSOR UNIT BEGIN!");
}

/**
 * Callback function when sensor information is requested
 */
void requestSensorInfo(){
  SensorData data = { vOut, distance };
  Wire.write((byte *)&data, sizeof data);
//  Serial.println("R => V: " + String(vOut) + " / D: " + String(distance) + " (" + sizeof(data) + ")");
}

/********
 * LOOP *
 ********/
void loop() {
  // capture sensor data
  SensorData sensorData = captureSensorData();

  // check sensor values if override is needed
  boolean isSensorOverride = false; // true or false
  if ((sensorData.distance >= STOP_DISTANCE_MIN && sensorData.distance <= STOP_DISTANCE_MAX) || (sensorData.distance <= REVERSE_DISTANCE_MAX)){
    isSensorOverride = true;
  }

  // capture HC-12 transmission
  while (hc12.available()){
    incomingByte = hc12.read();
    readBuffer += char(incomingByte);
    
    if (incomingByte == DELIMITER_END){
      txEnd = true;
//      Serial.println(readBuffer);
    } else if (incomingByte == DELIMITER_START) {
      txEnd = false;
//      readBuffer = "";
    }
  }

  // parse the transmission
  int dir = 0;
  int spd = 0;
  int durationMs = 0;
  if (txEnd && readBuffer != ""){
    int i;
    unsigned long rxMsg = 0;
    
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
      DriveCommand driveCommand;
      rxMsg = atol(commandList[i].substring(1,12).c_str());
      driveCommand.dir = (rxMsg & 0b00001110000000000000000000000000) >> 25; 
      driveCommand.spd = (rxMsg & 0b00000001111111100000000000000000) >> 17; 
      driveCommand.durationMs = (rxMsg & 0b00000000000000011111111111111000) >> 3; 

      // if the commands need to be overridden
      if (isSensorOverride){
        driveCommand = overrideCommand(sensorData);
      }

      // send the commands to the drive unit
      sendCommand(driveCommand);
    }

    readBuffer = "";
    txEnd = false;
  } else if (isSensorOverride) {
    DriveCommand driveCommand = overrideCommand(sensorData);
    sendCommand(driveCommand);
    isSensorOverride = false;
  }

  // every [DRIVE_POLL_MS] milliseconds, get the drive data
  currMillis = millis();
  DriveData driveData;
  if ((currMillis - prevMillis) > DRIVE_POLL_MS){
    prevMillis = currMillis;
    driveData = requestDriveData();
  }

  // send back telemetry to tx_unit
  Telemetry telemetry = {driveData, sensorData};
//  sendTelemetry(telemetry);
} 

/*********************
 * PRIVATE FUNCTIONS *
 *********************/

/**
 * Send back telemetry information to tx_unit
 */
void sendTelemetry(Telemetry telemetry){
//  Serial.println("(" + String(sizeof(telemetry)) + ") spd: " + String(telemetry.drive.spd) + " / v: " + String(telemetry.sensor.voltage) + " / d: " + String(telemetry.sensor.distance));
  hc12.write((byte *)&telemetry, sizeof telemetry);
//  hc12.print("test");
}

/**
 * Request DriveData from drive_unit
 */
DriveData requestDriveData(){
  DriveData driveData;
//  driveData.spd = 0;
  Wire.requestFrom(DRIVE_SUBSYS_ADDR, DRIVE_DATA_ANSSIZE);
  Wire.readBytes((byte *)&driveData, DRIVE_DATA_ANSSIZE);
//  Serial.println("SPD: " + String(driveData.spd));
  return driveData;
}

/**
 * Override the command if certain thresholds are reached
 */
DriveCommand overrideCommand(SensorData sensorData){
//  Serial.println("overrideCommand");
  DriveCommand driveCommand;
  
  driveCommand.spd = 255;
  driveCommand.durationMs = 0;
  driveCommand.dir = 0;
  
  if (sensorData.distance >= STOP_DISTANCE_MIN && sensorData.distance <= STOP_DISTANCE_MAX){
    driveCommand.dir = 0; // stop if STOP_DISTANCE is reached
  } else if (sensorData.distance <= REVERSE_DISTANCE_MAX){
    driveCommand.dir = 2; // reverse if REVERSE_DISTANCE is reached
  }

  return driveCommand;
}

/**
 * Transmit DriveCommand to drive unit
 */
void sendCommand(DriveCommand driveCommand){
  Serial.println("sendCommand dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
  Wire.beginTransmission(DRIVE_SUBSYS_ADDR);
  Wire.write((byte *)&driveCommand, sizeof driveCommand);
  Wire.endTransmission();
}

/**
 * Get sensor data and build the structure
 */
SensorData captureSensorData(){
  SensorData sensorData;

//  sensorData.voltage = 0.0;
//  sensorData.distance = 0.0;
  sensorData.voltage = mapFloat(analogRead(V_SENSOR), VIN_MIN, VIN_MAX, VOUT_MIN, VOUT_MAX);
  sensorData.distance = pingHCSR04(); 
  
//  Serial.println("V: " + String(sensorData.voltage) + " / D: " + String(sensorData.distance));

  return sensorData;
}

/**
 * Map function for float values
 */
float mapFloat(float in, float x1, float x2, float y1, float y2){
  float m = (y2 - y1) / (x2 - x1);
  return (m * (in - x1)) + y1;
}

/**
 * Send a pulse on the HC-SR04 and get the distance
 */
float pingHCSR04(){
  // create a wave ping
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(HC_SR04_PING_US);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(HC_SR04_PING_US);
  digitalWrite(HCSR04_TRIG, LOW);

  float pingUs = pulseIn(HCSR04_ECHO, HIGH);

  delay(HC_SR04_WAIT_US);

  float distance = (MPS * (pingUs / 1000000)) / 2; // distance = rate * time

  return distance;
}
