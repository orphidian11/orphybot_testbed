/**
 * OrphyBot Test Bed
 * Transmitter unit (Arduino NANO)
 * Capture joystick signals and translate into binary transmissions
 *  
 * Connected Devices:
 * - (A5: x-axis, A4: y-axis, D7: button) Joystick
 * - (D8: CE, D10:CSN, D13: SCK, D11: MOSI, D12: MISO) NRF24L01 Transceiver
 */

#include <RHReliableDatagram.h>
#include <SPI.h>
#include <RH_NRF24.h>

#define NRF24L01_CHANNEL 10
#define LEADER_ADDRESS 1
#define FOLLOWER_ADDRESS 2

// pinouts
#define JX A5
#define JY A4
#define JZ 7

/** CONSTANTS **/
// wheels
const int L = 0;
const int R = 1;

// speed range
const float MIN_SPD = 0;
const float MAX_SPD = 255;
const float START_SPD = 80;

// joystick range
const float J_MIN = 0;
const float J_MAX = 1023;

// joystick neutral range tolerance 
const float J_NEUT_MIN = 480;
const float J_NEUT_MAX = 520;
const unsigned long DELIM_START = 0b1100;
const unsigned long DELIM_END = 0b011;
const char DELIMITER_START = '['; 
const char DELIMITER_END = ']'; 
const int MSG_LENGTH = 12;
const int COMMAND_LIST_SIZE = 10; // maximum of 10 commands can be sent
const int TELEMETRY_SIZE = 10;
const int RESPONSE_TIMEOUT = 100;
const int TX_DELAY_MS = 100;

// inputs
int xVal;
int yVal;

// transmit output
unsigned long dir = 0; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
unsigned long prevDir = dir;
unsigned long spd = 0; // 0 to 255
unsigned long durationMs = 0;
char txString[12];
boolean txEnd = false;
byte incomingByte;
String readBuffer = "";
int currMs = 0;
int prevMs = 0;

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
  int volt; // [0-255]
  int amps; // [0-255]
  unsigned long distPingUs; // ping duration in microseconds (4 bytes)
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
  // put your setup code here, to run once:
  pinMode(JX, INPUT);
  pinMode(JY, INPUT);
  pinMode(JZ, INPUT_PULLUP);

  // init nrf24l01
  if (!nrf24.init()) Serial.println("init failed");
  if (!nrf24.setChannel(NRF24L01_CHANNEL)) Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) Serial.println("setRF failed"); 
//  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm6dBm)) Serial.println("setRF failed"); 
  
  Serial.begin(9600);
  Serial.println("TX BEGIN!");

  currMs = millis();

//  int x = analogRead(JX);
//  uint8_t arr[2] = {highByte(x), lowByte(x)};
//  int y = ((int) arr[0] << 8) | arr[1];
//  Serial.println(String(x) + " >> " + String(y));
}

/********
 * LOOP *
 ********/
 
void loop() {
  /** INPUT **/
  DriveCommand cmd = captureCommands();

  // TODO: capture any manual commands

  // transmit joystick commands
  transmitCommands(cmd);
}

/*********************
 * PRIVATE FUNCTIONS *
 *********************/
 
/**
 * Transmit the commands
 */
void transmitCommands(DriveCommand driveCommand){
  unsigned long beginMs = millis();
  
  // command to be sent (break 
  uint8_t txCmd[8]; 
  txCmd[0] = highByte(driveCommand.x); 
  txCmd[1] = lowByte(driveCommand.x); 
  txCmd[2] = highByte(driveCommand.y); 
  txCmd[3] = lowByte(driveCommand.y); 
  txCmd[4] = highByte(driveCommand.spd);
  txCmd[5] = lowByte(driveCommand.spd);
  txCmd[6] = driveCommand.sw;
  txCmd[7] = driveCommand.durationMs;

  nrf24.send(txCmd, sizeof(txCmd));
  nrf24.waitPacketSent();
  
//  Serial.print("SEND >> "); 
//  Serial.print("x: " + String(driveCommand.x) + " / ");
//  Serial.print("y: " + String(driveCommand.y) + " / ");
//  Serial.print("sw: " + String(driveCommand.sw) + " / ");
//  Serial.print("spd: " + String(driveCommand.spd) + " / "); 
//  Serial.print("durationMs: " + String(driveCommand.durationMs));
//  Serial.println("(" + String(millis() - beginMs) + "ms)");
  
  if (nrf24.waitAvailableTimeout(RESPONSE_TIMEOUT)){
    // receive telemetry
    receiveTelemetry();
  }
}

/**
 * Receive telemetry from SENSOR unit
 */
void receiveTelemetry(){
  unsigned long beginMs = millis();
  
  // variable for holding response
  // expected structure: 0: speed, 1: distance, 2: voltage, 3: current
  uint8_t resp[RH_NRF24_MAX_MESSAGE_LEN];
  uint8_t respLen = sizeof(resp);
  uint8_t from;
  
  if (nrf24.available()){
    if (nrf24.recv(resp, &respLen)){
      Telemetry telemetry = readTelemetry(resp);
    }
  }
}

/**
 * Read telemtry data from the array
 */
Telemetry readTelemetry(uint8_t resp[]){
  Telemetry telemetry;
  
  telemetry.drive.spd = ((int) resp[0] << 8) | resp[1]; 
  telemetry.sensor.distPingUs = resp[2] << 24 | resp[3] << 16 | resp[4] << 8 | resp[5];
  telemetry.sensor.volt = ((int) resp[6] << 8) | resp[7]; 
  telemetry.sensor.amps = ((int) resp[8] << 8) | resp[9]; // current 
  
  Serial.print("RECV << ");
  Serial.print("spd: " + String(telemetry.drive.spd) + " / ");
  Serial.print("dis: " + String(telemetry.sensor.distPingUs) + " / ");
  Serial.print("volt: " + String(telemetry.sensor.volt) + " / ");
  Serial.println("amps: " + String(telemetry.sensor.amps) + " ");

  return telemetry;
}

/**
 * Capture and build command object
 */
DriveCommand captureCommands(){
  DriveCommand cmd;
  
  cmd.x = analogRead(JX);
  cmd.y = analogRead(JY);
  cmd.sw = !digitalRead(JZ);
  cmd.spd = 255; // raw data from potentiometer
  cmd.durationMs = 0;

//  Serial.println(String(analogRead(JX)) + ">>" + String(cmd.x));

  return cmd;
}
