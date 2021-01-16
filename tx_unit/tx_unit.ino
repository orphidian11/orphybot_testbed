/**
 * OrphyBot Test Bed
 * Transmitter unit (Arduino NANO)
 * Capture joystick signals and translate into binary transmissions
 *  
 * Connected Devices:
 * - (A5: x-axis, A4: y-axis, D7: button) Joystick
 * - (D8: CE, D10:CSN, D13: SCK, D11: MOSI, D12: MISO) NRF24L01 Transceiver
 * 
 * Format: (32 bits)
 *  0x0FFFFFF8
 *  0b0000 111 11111111 11111111111111 000
 * 
 * Where: 
 *  0b1100 - (4 bits) begin
 *  0b111 - (3 bits) direction (0b000 - stop; 0b001 - forward; 0b010 - reverse; 0b011 - left; 0b100 - right)
 *  0b11111111 - (8 bits) speed (0 to 255)
 *  0b11111111111111 - (14 bits) duration in milliseconds (0 to 16383 milliseconds)
 *  0b011 - (3 bits) end
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

// sensor data structure
struct SensorData {
  float voltage; 
  float distance; // in meters
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
  pinMode(JZ, INPUT);

  // init nrf24l01
  if (!nrf24.init()) Serial.println("init failed");
  if (!nrf24.setChannel(NRF24L01_CHANNEL)) Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) Serial.println("setRF failed"); 
//  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPowerm6dBm)) Serial.println("setRF failed"); 
  
  Serial.begin(9600);
  Serial.println("TX BEGIN!");

  currMs = millis();
}

/********
 * LOOP *
 ********/
 
void loop() {
  unsigned long beginMs = millis();
//  Serial.println("!!");

  // get the previous direction
  prevDir = dir;

  /** INPUT **/
  xVal = analogRead(JX);
  yVal = analogRead(JY);

  /** PROCESS **/
  // direction
  // for this version, forward/reverse takes precedence over left/right
  if (yVal <= J_NEUT_MIN){ // forward
    dir = 1;
  } else if (yVal >= J_NEUT_MAX){ // reverse
    dir = 2;
  } else if (xVal <= J_NEUT_MIN){ // left turn
    dir = 3;
  } else if (xVal >= J_NEUT_MAX) { // right turn
    dir = 4;
  } else { // stop
    dir = 0;
  }

  // speed
  if (dir != 0){
    spd = 255;
  } else {
    spd = 0;
  }

  // duration
  durationMs = 0; // for now, always 0

  // capture any manual commands
  // format: [x-yyy-zzzz]
  // where: 
  //  x = direction
  //  yyy = speed
  //  zzzz = duration
//  while(hc12.available()){
//    Serial.println("test");
//    incomingByte = hc12.read();
//    readBuffer += char(incomingByte);
//    
//    if (incomingByte == DELIMITER_END){
//      txEnd = true;
//      Serial.println(readBuffer);
//    } else if (incomingByte == DELIMITER_START) {
//      txEnd = false;
//      readBuffer = "";
//    }
//  }

  // transmit joystick commands
  transmitCommands(dir, spd, durationMs);
}

/*********************
 * PRIVATE FUNCTIONS *
 *********************/
 
/**
 * Transmit the commands
 * @param d direction
 * @param s speed
 * @param ms duration in milliseconds
 */
void transmitCommands(unsigned long d, unsigned long s, unsigned long ms){
  unsigned long beginMs = millis();
  
  // command to be sent
  uint8_t cmd[3]; 
  cmd[0] = dir; 
  cmd[1] = spd; 
  cmd[2] = durationMs; 

  nrf24.send(cmd, sizeof(cmd));
  nrf24.waitPacketSent();
  if (nrf24.waitAvailableTimeout(RESPONSE_TIMEOUT)){
    Serial.println("SEND >> dir: " + String(d) + " / spd: " + String(s) + " / durationMs: " + String(ms) + " (" + String(millis() - beginMs) + "ms)");
  
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
      Serial.println("RECV << spd: " + String(resp[0]) + " / dis: " + String(resp[1]) + " / volt: " + String(resp[2]) + " / amps: " + String(resp[3]) + " (" + String(millis() - beginMs) + "ms)"); 
    }
  }
}
