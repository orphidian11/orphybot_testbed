/**
 * OrphyBot Test Bed
 * 
 * Blink Program
 * Blink the LEDs to test connectivity
 */

#define RED_PIN 9
#define GRN_PIN 8

const int loopDelayMs = 1000;
const int blinkDelayMs = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(RED_PIN, OUTPUT);
  pinMode(GRN_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  // blink red twice
  digitalWrite(RED_PIN, HIGH);
  delay(blinkDelayMs);
  digitalWrite(RED_PIN, LOW);
  delay(blinkDelayMs);
  digitalWrite(RED_PIN, HIGH);
  delay(blinkDelayMs);
  digitalWrite(RED_PIN, LOW);

  // delay before blinking green
  delay(loopDelayMs);

  // blink green twice
  digitalWrite(GRN_PIN, HIGH);
  delay(blinkDelayMs);
  digitalWrite(GRN_PIN, LOW);
  delay(blinkDelayMs);
  digitalWrite(GRN_PIN, HIGH);
  delay(blinkDelayMs);
  digitalWrite(GRN_PIN, LOW);

  // delay before looping again
  delay(loopDelayMs);

}
