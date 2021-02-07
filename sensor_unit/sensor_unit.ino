/**
 * OrphyBot Test Bed
 * Sensor Unit (Arduino NANO)
 * Capture signals from sensors and send to Drive Unit via I2C
 * 
 * Connected Devices:
 * - (A4:SDA, A5:SCL) Arduino UNO as follower unit via I2C 
 * - (A3) Voltage Sensor 
 * - (D2:Trig, D3:Echo) HC-SR04 Ultrasonic Distance Sensor
 * - (D8: CE, D10:CSN, D13: SCK, D11: MOSI, D12: MISO) NRF24L01 Transceiver
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
 
#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <Wire.h>

#define NRF24L01_CHANNEL 10
#define LEADER_ADDRESS 1
#define FOLLOWER_ADDRESS 2
#define DRIVE_SUBSYS_ADDR 8
#define DRIVE_DATA_ANSSIZE 2
#define SENSORS_SUBSYS_ADDR 9
#define SENSORS_SUBSYS_ANSSIZE 8 // number of characters

// pinouts
#define V_SENSOR A3
#define HCSR04_TRIG 2
#define HCSR04_ECHO 3
#define LED_PIN 13 

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

// strcture for commands to be transmitted
struct DriveCommand {
  int x; // joystick x-axis
  int y; // joystick y-axis
  int sw; // joystick switch
  int spd; // speed
  int durationMs; // duration in milliseconds
};

// sensor data structure
struct SensorData {
  int volt; 
  int amps;
  int distPingUs; // ping duration in microseconds
};

// drive data structure 
struct DriveData {
  int spd;
};

// telemetry structure 
struct Telemetry {
  DriveData drive;
  SensorData sensor;
};

RH_NRF24 nrf24;

/*********
 * SETUP *
 *********/
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(V_SENSOR, INPUT);
  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);
  
  Wire.begin(); // run in leader mode
  
  // init nrf24l01
  if (!nrf24.init()) Serial.println("init failed");
  if (!nrf24.setChannel(NRF24L01_CHANNEL)) Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) Serial.println("setRF failed"); 
//  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm6dBm)) Serial.println("setRF failed"); 

  Serial.begin(9600);
  Serial.println("SENSOR UNIT BEGIN!");
}

/********
 * LOOP *
 ********/
void loop() {
  unsigned long beginMs = millis();  

  // capture sensor data
  SensorData sensorData = captureSensorData();

  // check sensor values if override is needed
  boolean isSensorOverride = false; // true or false
//  if ((sensorData.distPingUs >= STOP_DISTANCE_MIN && sensorData.distPingUs <= STOP_DISTANCE_MAX) || (sensorData.distance <= REVERSE_DISTANCE_MAX)){
//    isSensorOverride = true;
//  }

  receiveCommand(isSensorOverride, sensorData);
} 

/*********************
 * PRIVATE FUNCTIONS *
 *********************/

/**
 * Receive transmission from TX unit
 */
void receiveCommand(boolean isSensorOverride, SensorData sensorData){
  unsigned long beginMs = millis();
  
  // varriable for holding command
  // expected structure: 0: direction, 1: speed, 2: duration
  uint8_t cmd[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t cmdLen = sizeof(cmd);

  // capture transmission
  if (nrf24.available()){
    if (nrf24.recv(cmd, &cmdLen)){
      DriveCommand driveCommand;
      driveCommand.x = ((int) cmd[0] << 8) | cmd[1]; 
      driveCommand.y = ((int) cmd[2] << 8) | cmd[3]; 
      driveCommand.spd = ((int) cmd[4] << 8) | cmd[5]; 
      driveCommand.sw = cmd[6]; 
      driveCommand.durationMs = cmd[7];
      
      Serial.print("RECV << "); 
      Serial.print("x: " + String(driveCommand.x) + " / ");
      Serial.print("y: " + String(driveCommand.y) + " / ");
      Serial.print("sw: " + String(driveCommand.sw) + " / ");
      Serial.print("spd: " + String(driveCommand.spd) + " / "); 
      Serial.print("durationMs: " + String(driveCommand.durationMs));
      Serial.println("(" + String(millis() - beginMs) + "ms)");

      // if the commands need to be overridden
//      if (isSensorOverride){
//        driveCommand = overrideCommand(sensorData);
//      }

      // every [DRIVE_POLL_MS] milliseconds, get the drive data
      currMillis = millis();
      DriveData driveData;
      if ((currMillis - prevMillis) > DRIVE_POLL_MS){
        prevMillis = currMillis;
        driveData = requestDriveData();
      }
      
      // send the commands to the drive unit
      sendCommand(driveCommand);
      
      // send back telemetry to tx_unit
//      Telemetry telemetry = {driveData, sensorData};
//      sendTelemetry(telemetry);
    }
  }
}

/**
 * Send back telemetry information to tx_unit
 */
void sendTelemetry(Telemetry telemetry){
  unsigned long beginMs = millis();
  
  // response to be sent
  uint8_t resp[4];
  resp[0] = telemetry.drive.spd; 
  resp[1] = telemetry.sensor.distPingUs; 
  resp[2] = telemetry.sensor.volt; 
  resp[3] = telemetry.sensor.amps; // current 

  nrf24.send(resp, sizeof(resp));
  nrf24.waitPacketSent();
  
//  Serial.print("SEND >> ");
//  Serial.print("spd: " + String(telemetry.drive.spd) + " / ");
//  Serial.print("dis: " + String(telemetry.sensor.distPingUs) + " / ");
//  Serial.print("volt: " + String(telemetry.sensor.volt) + " / ");
//  Serial.println("amps: " + String(telemetry.sensor.amps) + " ");
}

/**
 * Request DriveData from drive_unit
 */
DriveData requestDriveData(){
  DriveData driveData;
  driveData.spd = 0;
//  Wire.requestFrom(DRIVE_SUBSYS_ADDR, DRIVE_DATA_ANSSIZE);
//  Wire.readBytes((byte *)&driveData, DRIVE_DATA_ANSSIZE);
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
//  driveCommand.x = 0;
//  driveCommand.y = 0;
  
//  if (sensorData.distance >= STOP_DISTANCE_MIN && sensorData.distance <= STOP_DISTANCE_MAX){
//    driveCommand.dir = 0; // stop if STOP_DISTANCE is reached
//  } else if (sensorData.distance <= REVERSE_DISTANCE_MAX){
//    driveCommand.dir = 2; // reverse if REVERSE_DISTANCE is reached
//  }

  return driveCommand;
}

/**
 * Transmit DriveCommand to drive unit
 */
void sendCommand(DriveCommand driveCommand){
//  Serial.println("sendCommand dir: " + String(driveCommand.dir) + " / spd: " + String(driveCommand.spd) + " / durationMs: " + String(driveCommand.durationMs));
  Wire.beginTransmission(DRIVE_SUBSYS_ADDR);
  Wire.write((byte *)&driveCommand, sizeof driveCommand);
  Wire.endTransmission();
  delay(1);
}

/**
 * Get sensor data and build the structure
 */
SensorData captureSensorData(){
  SensorData sensorData;

  sensorData.volt = 0; // analogRead(V_SENSOR);
  sensorData.distPingUs = 0; // pingHCSR04(); 
  sensorData.amps = 0; 
  
//  Serial.println("V: " + String(sensorData.volt) + " / D: " + String(sensorData.distPingUs) + " / A: " + String(sensorData.amps));

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
uint8_t pingHCSR04(){
  // create a wave ping
  digitalWrite(HCSR04_TRIG, LOW);
  delayMicroseconds(HC_SR04_PING_US);
  digitalWrite(HCSR04_TRIG, HIGH);
  delayMicroseconds(HC_SR04_PING_US);
  digitalWrite(HCSR04_TRIG, LOW);

  uint8_t distPingUs = pulseIn(HCSR04_ECHO, HIGH);

  delay(HC_SR04_WAIT_US);

  return distPingUs;
}

/**
 * Compute the distance captured from HC-SR04
 */
float computeDistPing(uint8_t distPingUs){
  float distance = (MPS * (distPingUs / 1000000)) / 2; // distance = rate * time
  return distance;
}
