/**
 * OrphyBot Test Bed
 * 
 * OrphyBot System
 * Follower Sub-system - Sensors
 * Receive requests via I2C from Leader sub-system and 
 * transmit sensor information
 */
 
#include<Wire.h>

// I2C followers address
#define SENSORS_SUBSYS 9
#define SENSORS_SUBSYS_ANSSIZE 6 // number of characters

#define HCSR04_TRIG_PIN 2
#define HCSR04_ECHO_PIN 3

// constants
const int mps = 343; // speed of sound (m/s)

// inputs
float pingUs; // travel time of ping (microseconds)
float distance = 0; // in meters

void setup() {
  // put your setup code here, to run once:
  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  
  Wire.begin(SENSORS_SUBSYS); // run in follower mode

  // event handler function for onRequest event
  Wire.onRequest(requestSensorInfo);

  Serial.begin(9600);
}

/**
 * Callback function when sensor information is requested
 */
void requestSensorInfo(){
  byte response[SENSORS_SUBSYS_ANSSIZE];
  char distanceStr[6];
  
  dtostrf(distance, 3, 2, distanceStr);

  // convert the answer into an array
  for (byte i = 0; i < SENSORS_SUBSYS_ANSSIZE; i++){
    response[i] = (byte) distanceStr[i];
  }

  // send response to master
  Wire.write(response, sizeof(response));
}

void loop() {
  // put your main code here, to run repeatedly:
  distance = captureDistance();
  Serial.println(distance);
}

/**
 * Capture distance using HC-SR04 sensor
 * @return float distance in meters
 */
float captureDistance(){
  // create a wave ping
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, LOW);

  pingUs = pulseIn(HCSR04_ECHO_PIN, HIGH);

  delay(25);

  float dis = (mps * (pingUs / 1000000)) / 2; // distance = rate * time

  return dis; 
}
