/**
 * OrphyBot Test Bed
 * 
 * Binary transmissions
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
const float minSpd = 0;
const float maxSpd = 255;
const float startSpd = 80;

// joystick range
const float jMin = 0;
const float jMax = 1023;

// joystick neutral range tolerance 
const float jNeutMin = 480;
const float jNeutMax = 520;
const int txDelayMs = 100;

const unsigned long delimiterStart = 0b1100;
const unsigned long delimiterEnd = 0b011;

// inputs
int xVal;
int yVal;

// transmit output
unsigned long txMsg = 0; // 32 bit message
unsigned long dir = 0; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
unsigned long prevDir = dir;
unsigned long spd = 0; // 0 to 255
unsigned long durationMs = 0;
char txString[12];
boolean txEnd = false;
byte incomingByte;
String readBuffer = "";

void setup() {
  // put your setup code here, to run once:
  pinMode(JX, INPUT);
  pinMode(JY, INPUT);

  Serial.begin(9600);
  Serial.println("TX BEGIN!");
}

void loop() {
  /*************TEST***************/
//  Serial.println("Input dir [0-4]:");
//  while (Serial.available() == 0);
//  dir = Serial.parseInt();  
//  Serial.println(dir, BIN);
//
//  Serial.println("Input spd [0-255]:");
//  while (Serial.available() == 0);
//  spd = Serial.parseInt();  
//  Serial.println(spd, BIN);
//
//  Serial.println("Input durationMs [0-9999]:");
//  while (Serial.available() == 0);
//  durationMs = Serial.parseInt();  
//  Serial.println(durationMs, BIN);
//
//  txMsg = delimiterStart<<28 | dir<<25 | spd<<17 | durationMs<<3 | delimiterEnd;
//  
//  // expected output shoud be
//  // 0b0000 001 11111111 01001110001000 000
//  Serial.println(txMsg, BIN);
//
//  Serial.println("Press any key to continue...");
//  while (Serial.available() == 0);
  /***********END TEST*************/
  
  // get the previous direction
  prevDir = dir;

  /** INPUT **/
  xVal = analogRead(JX);
  yVal = analogRead(JY);

  /** PROCESS **/
  // direction
  // for this version, forward/reverse takes precedence over left/right
  if (yVal <= jNeutMin){ // forward
    dir = 1;
  } else if (yVal >= jNeutMax){ // reverse
    dir = 2;
  } else if (xVal <= jNeutMin){ // left turn
    dir = 3;
  } else if (xVal >= jNeutMax) { // right turn
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
//  while(Serial.available()){
//    incomingByte = Serial.read();
//    readBuffer += char(incomingByte);
//    
//    if (incomingByte == delimiterEnd){
//      txEnd = true;
//    } 
//  }
//  if (txEnd && !readBuffer.equals("")){
//    Serial.print(readBuffer);
//    readBuffer = "";
//    txEnd = false;
//  }

  // transmit only if the direction is not stop, 
  // or if it is stop, but the previous loop's dir is not stop
  // this is to prevent constant transmit if it is stop
  if (dir != 0 || (dir == 0 && prevDir != dir)){
    // convert to string 
    sprintf(txString, "[%d-%03d-%04d]", dir, spd, durationMs);
    txMsg = delimiterStart<<28 | dir<<25 | spd<<17 | durationMs<<3 | delimiterEnd;
  
    // output
//    Serial.println("[" + String(xVal) + "," + String(yVal) + "] => " + txString);
//    Serial.println(txString);
//    Serial.println(txMsg, BIN);
//    Serial.print(txString);
//    Serial.print(txMsg, BIN);
    Serial.print(txMsg);
    delay(txDelayMs);
  }
}
