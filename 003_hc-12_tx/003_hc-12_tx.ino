/**
 * OrphyBot Test Bed
 * 
 * Capture input from potentiometer and transmit to OrphyBot via HC-12
 * 
 * Format of Input Transmitted Thru HC-12:
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
const float jNeutMax = 510;
const int txDelayMs = 100;

// inputs
int xVal;
int yVal;
//float rSpd = minSpd;
//float lSpd = minSpd;
//float lSpdRedux = minSpd;
//float rSpdRedux = minSpd;
//int phaseInterval = 10000;
//int rampUpInterval = 20;

// transmit output
int dir = 0; // 0 - stop; 1 - forward; 2 - reverse; 3 - left; 4 - right
int spd = 0; // 0 to 255
int durationMs = 0;
char txString[12];

void setup() {
  // put your setup code here, to run once:
  pinMode(JX, INPUT);
  pinMode(JY, INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  /** INPUT **/
  int xVal = analogRead(JX);
  int yVal = analogRead(JY);

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

  // convert to string 
  sprintf(txString, "[%d-%03d-%04d]", dir, spd, durationMs);

  // output
//  Serial.println("[" + String(xVal) + "," + String(yVal) + "] => " + txString);
  Serial.print(txString);
  delay(txDelayMs);
}
