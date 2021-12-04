/**
 * Orphybot Controller v2.0
 */

#include<Wire.h>

// I2C followers address
#define I2C_CTRL_ADD 11
#define I2C_ARM_ADD 4 

// pin setup 
#define L_X_PIN A1
#define L_Y_PIN A0
#define L_BTN_PIN 2
#define R_X_PIN A3
#define R_Y_PIN A2
#define R_BTN_PIN 3

const int L_X_MIN_NEUT = 500; // 530; // 500
const int L_X_MAX_NEUT = 520; // 550; // 520
const int L_Y_MIN_NEUT = 500; // 530; // 500
const int L_Y_MAX_NEUT = 520; // 550; // 520
const int R_X_MIN_NEUT = 505; // 540; // 505
const int R_X_MAX_NEUT = 525; // 555; // 525
const int R_Y_MIN_NEUT = 500; // 530; // 500
const int R_Y_MAX_NEUT = 520; // 550; // 520

const int X_MIN_LIMIT = 0; // mm
const int Y_MIN_LIMIT = 0; // mm
const int X_MAX_LIMIT = 130; // mm
const int Y_MAX_LIMIT = 100; // mm
const int XYZ_INCREMENTS = 1; // mm

const int DEG_MIN_LIMIT = 0; // degrees
const int DEG_MAX_LIMIT = 360; // degrees
const int DEG_INCREMENTS = 1; // degrees

struct InputValues {
  int lX;
  int lY;
  int lBtn;

  int rX;
  int rY;
  int rBtn;
};

struct Coords {
  int x;
  int y;
  int z;
  int deg; // 0 - 360 degrees
  bool lBtn;
  bool rBtn;
};

Coords coords = Coords();

void setup() {
  Serial.begin(9600);
  
  pinMode(L_X_PIN, INPUT);
  pinMode(L_Y_PIN, INPUT);
  pinMode(L_BTN_PIN, INPUT_PULLUP);
  
  pinMode(R_X_PIN, INPUT);
  pinMode(R_Y_PIN, INPUT);
  pinMode(R_BTN_PIN, INPUT_PULLUP);

  // initialize coords
  // TODO: add startup calibration here for neutral values
  resetCoords();
  
  Wire.begin(); // leader mode

  Serial.println("*** Orphybot Controller starting... ***");
}

byte x = 0;

void loop() {
  // read controller inputs
  InputValues input = readValues();
  
  // convert values
  convertValues(input);

  // transmit values
  transmitValues();

  Serial.println("");
}

/**
 * Read sensor values
 */
InputValues readValues(){
  InputValues input = InputValues();

  input.lX = analogRead(L_X_PIN);
  input.lY = analogRead(L_Y_PIN);
  input.lBtn = digitalRead(L_BTN_PIN);

  input.rX = analogRead(R_X_PIN);
  input.rY = analogRead(R_Y_PIN);
  input.rBtn = digitalRead(R_BTN_PIN);

  Serial.print("[lx,ly,btn],[rx,ry,btn]: [" 
    + String(input.lX) + "," + String(input.lY) + "," + String(input.lBtn) 
    + "],[" 
    + String(input.rX) + "," + String(input.rY) + "," + String(input.rBtn) + "]");

  return input;
}

/**
 * Read the inputs and update the coords
 */
void convertValues(InputValues input){
  // x-axis
  if (input.lX <= L_X_MIN_NEUT && coords.x > X_MIN_LIMIT){
    coords.x -= XYZ_INCREMENTS;
  } else if (input.lX >= L_X_MAX_NEUT && coords.x < X_MAX_LIMIT) {
    coords.x += XYZ_INCREMENTS;
  }

  // y-axis
  if (input.lY <= L_Y_MIN_NEUT && coords.y > Y_MIN_LIMIT){
    coords.y -= XYZ_INCREMENTS;
  } else if (input.lY >= L_Y_MAX_NEUT && coords.y < Y_MAX_LIMIT) {
    coords.y += XYZ_INCREMENTS;
  }

  // rotation
  if (input.rX <= R_X_MIN_NEUT && coords.deg > DEG_MIN_LIMIT){
    coords.deg -= DEG_INCREMENTS;
  } else if (input.rX >= R_X_MAX_NEUT && coords.deg < DEG_MAX_LIMIT) {
    coords.deg += DEG_INCREMENTS;
  }

  // left button
  coords.lBtn = (input.lBtn == 0);
  
  // right button
  coords.rBtn = (input.rBtn == 0);

  // reset coordinates
  if (coords.lBtn == 1 && coords.lBtn == 1){
    resetCoords();
  }

  String txt;
  if (coords.lBtn == true){
    txt = "true";
  } else {
    txt = "false";
  }
  Serial.print(" => [x,y,deg,lbtn,rbtn]: [" + 
    String(coords.x) + "," + 
    String(coords.y) + "," + 
    String(coords.deg) + "," +
    txt + "," +
    // String(coords.lBtn) + "," +
    String(coords.rBtn) + "]");
}

/**
 * Return coordinates to initial values
 */
void resetCoords(){
  coords.x = X_MAX_LIMIT / 2;
  coords.y = Y_MAX_LIMIT / 2;
  coords.z = 0;
  coords.deg = DEG_MAX_LIMIT / 2;
  coords.lBtn = false;
  coords.rBtn = false;
}

/**
 * Send the values via i2c
 */
void transmitValues(){
  Wire.beginTransmission(I2C_ARM_ADD);
  Wire.write((byte *) &coords, sizeof(coords));
  Wire.endTransmission();

  Serial.print(" [TRANSMITTED!]");
  delay(100);
}
