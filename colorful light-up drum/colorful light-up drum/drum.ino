#include <Adafruit_NeoPixel.h>
#include <LowPower.h>

int LED_PIN = 3; // The pin that the NeoPixel is connected to
int Sensor_PIN = A2;
int WAKEUP_PIN = 2; // The pin that will wake up the device
int NUM_LEDS = 256; // The number of NeoPixels
int MIN_LED_COUNT = 15; // The minimum number of LEDs to light up
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

static unsigned long lastActivityTime;
int rainbowIndex = 0;
int rainbowColors[][3] = {
  {255, 0, 0},    // Red
  {255, 127, 0},  // Orange
  {255, 255, 0},  // Yellow
  {0, 255, 0},    // Green
  {0, 0, 255},    // Blue
  {75, 0, 130},   // Indigo
  {148, 0, 211}   // Violet
};

int baselineValue = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  pixels.begin(); // Initialize NeoPixel
  pixels.show();
  pinMode(WAKEUP_PIN, INPUT); // Set the wakeup pin as input
  lastActivityTime = millis();

  // Read the sensor value for 3 seconds and take the average as baseline
  int sum = 0;
  int samples = 0;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 3) {
    int sensorValue = analogRead(Sensor_PIN);
    sum += sensorValue;
    samples++;
    delay(5); // Small delay to avoid oversampling
  }
  
  baselineValue = sum / samples;
  Serial.print("Baseline sensor value: ");
  Serial.println(baselineValue);
}

void loop() {
  int sensorValue = analogRead(Sensor_PIN); // Read the value from analog pin A2
  //Serial.println(sensorValue);
  //delay(5); // Delay for 5 milliseconds

  updateLeds(sensorValue); // Update the LEDs based on the sensor value

  // Shift the LEDs up by one position and turn off the top LED
  for (int i = NUM_LEDS - 1; i > 0; i--) {
    pixels.setPixelColor(i, pixels.getPixelColor(i - 1));
  }

  pixels.setPixelColor(0, 0); // Clear the first pixel
  pixels.show();

  // Enter sleep mode if no activity for more than 60 seconds
  if (millis() - lastActivityTime > 60000) {
    pixels.clear();
    pixels.show();
    Serial.println("Entering sleep mode...");
    attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), wakeup, RISING); // Attach interrupt to wakeup pin
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); // Enter sleep mode
    detachInterrupt(digitalPinToInterrupt(WAKEUP_PIN)); // Detach interrupt from wakeup pin
    lastActivityTime = millis(); // Reset the activity timer
  }

  // Reset activity timer on wakeup pin trigger
  if (digitalRead(WAKEUP_PIN) == HIGH) {
    lastActivityTime = millis(); // Reset the activity timer
  }
}

void wakeup() {
  lastActivityTime = millis(); // Reset the activity timer
}

void updateLeds(int sensorValue) {
  // Trigger if the sensor value exceeds twice the baseline
  if (sensorValue > 5 * baselineValue) {
    rainbowIndex++;
    if (rainbowIndex >= 7) {
      rainbowIndex = 0;
    }
    for (int i = 0; i < MIN_LED_COUNT; i++) {
      float per = 0.05 + (0.95 / MIN_LED_COUNT) * i;
      pixels.setPixelColor(i, pixels.Color(
        rainbowColors[rainbowIndex][0] * per, 
        rainbowColors[rainbowIndex][1] * per, 
        rainbowColors[rainbowIndex][2] * per));
    }
    pixels.show();
  }
}
