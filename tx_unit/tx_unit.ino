/**
 * OrphyBot Test Bed
 * Transmitter unit
 * Capture joystick signals and translate into binary transmissions
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

#define JX A0
#define JY A1

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
const int TX_DELAY_MS = 100;

const unsigned long DELIM_START = 0b1100;
const unsigned long DELIM_END = 0b011;

const int MSG_LENGTH = 12;
const int COMMAND_LIST_SIZE = 10; // maximum of 10 commands can be sent
const int TELEMETRY_SIZE = 10;

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

/*********
 * SETUP *
 *********/
 
void setup() {
  // put your setup code here, to run once:
  pinMode(JX, INPUT);
  pinMode(JY, INPUT);

  Serial.begin(9600);
  Serial.println("TX BEGIN!");
}

/********
 * LOOP *
 ********/
 
void loop() {
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
  while(Serial.available()){
    incomingByte = Serial.read();
    readBuffer += char(incomingByte);
    
    if (incomingByte == ']'){
      txEnd = true;
    } 
  }
  
  /** OUTPUT **/
  if (txEnd && !readBuffer.equals("")){
    // transmit manual commands
    String commandList[COMMAND_LIST_SIZE];
    int numberOfCommands = readBuffer.length() / MSG_LENGTH;
    int i;

    // break down the readBuffer into an array of commands
    for (i = 0; i < numberOfCommands; i++){
      commandList[i] = readBuffer.substring(0 + (i * MSG_LENGTH), MSG_LENGTH + (i * MSG_LENGTH));
    }

    // parse each command, then transmit
    for (i = 0; i < numberOfCommands; i++){
      dir = commandList[i].substring(1,2).toInt();
      spd = commandList[i].substring(3,6).toInt();
      durationMs = commandList[i].substring(7,11).toInt();
      
      transmit(dir, spd, durationMs);
    }

    readBuffer = "";
    txEnd = false;
  } else {
    // transmit joystick commands
    transmit(dir, spd, durationMs);
  }
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
void transmit(unsigned long d, unsigned long s, unsigned long ms){
  unsigned long txMsg = 0; // 32 bit message
  
  // transmit only if the direction is not stop, 
  // or if it is stop, but the previous loop's dir is not stop
  // this is to prevent constant transmit if it is stop
//  if (d != 0){
    // convert to string 
    txMsg = DELIM_START<<28 | d<<25 | s<<17 | ms<<3 | DELIM_END;
  
    // output
    Serial.print("[" + String(txMsg) + "]");
    delay(TX_DELAY_MS);
//  }
}
