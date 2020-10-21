/**
 * OrphyBot Test Bed
 * Sensor Unit
 * Capture signals from sensors and send to main arduino board via I2C
 * 
 * Sensors:
 * - (A5) Voltage Sensor 
 * - (D2:Trig, D3:Echo) HC-SR04 Ultrasonic Distance Sensor
 * 
 */
 
#include<Wire.h>

// I2C followers address
#define SENSORS_SUBSYS 9
#define SENSORS_SUBSYS_ANSSIZE 8 // number of characters
#define V_SENSOR A3
#define HCSR04_TRIG 2
#define HCSR04_ECHO 3

// constants
const float VIN_MIN = 0;
const float VIN_MAX = 1023;
const float VOUT_MIN = 0;
const float VOUT_MAX = 25;
const int MPS = 343; // speed of sound (m/s)
const int HC_SR04_PING_US = 10; // microsecond delay for HC-SR04 ping
const int HC_SR04_WAIT_US = 25; // microsecond delay for HC-SR04 delay

// output 
float distance;
float vOut;

// sensor data structure
struct SensorData {
  float voltage;
  float distance;
};

/************************BEGIN SETUP******************************/
void setup() {
  pinMode(V_SENSOR, INPUT);
  pinMode(HCSR04_TRIG, OUTPUT);
  pinMode(HCSR04_ECHO, INPUT);
  
  Wire.begin(SENSORS_SUBSYS); // run in follower mode
  Wire.onRequest(requestSensorInfo); // event handler function for onRequest event

  Serial.begin(9600);
  Serial.println("SENSOR BEGIN!");
}

/**
 * Callback function when sensor information is requested
 */
void requestSensorInfo(){
  SensorData data = { vOut, distance };
  Wire.write((byte *)&data, sizeof data);
//  Serial.println("R => V: " + String(vOut) + " / D: " + String(distance) + " (" + sizeof(data) + ")");
}
/************************END SETUP********************************/

/************************BEGIN LOOP*******************************/
void loop() {
  // capture input
  float vIn = analogRead(V_SENSOR);
  vOut = mapFloat(vIn, VIN_MIN, VIN_MAX, VOUT_MIN, VOUT_MAX);
  distance = pingHCSR04();
//  Serial.println("V: " + String(vOut) + " / D: " + String(distance));
}
/************************END LOOP*********************************/

/************************BEGIN PRIVATE FUNCTIONS******************/

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
/************************END PRIVATE FUNCTIONS********************/
