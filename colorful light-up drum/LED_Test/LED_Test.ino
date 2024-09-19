#include <Adafruit_NeoPixel.h>


int LED_PIN = 3; // The pin that the NeoPixel is connected to
int Sensor_PIN=A2;
int NUM_LEDS = 8; // The number of NeoPixels
int MIN_LED_COUNT = 3; // The minimum number of LEDs to light up
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
int rainbowIndex = 0;
int rainbowColors[][3] = {
  {255, 0, 0}, // Red
  {255, 127, 0}, // Orange
  {255, 255, 0}, // Yellow
  {0, 255, 0}, // Green
  {0, 0, 255}, // Blue
  {75, 0, 130}, // Indigo
  {148, 0, 211} // Violet
};

void setup() {
  Serial.begin(115200); // Initialize serial communication
  pixels.begin(); // Initialize NeoPixel
  pixels.show();
 
}

void loop() {
  int sensorValue = analogRead(Sensor_PIN); // Read the value from analog pin A2
  delay(5); // Delay for 20 milliseconds
  // Check if the device should enter sleep mode

 
  updateLeds(sensorValue); // Update the LEDs based on the sensor value

  // Shift the LEDs up by one position and turn off the top LED
  for (int i = NUM_LEDS - 1; i > 0; i--) {
    pixels.setPixelColor(i, pixels.getPixelColor(i - 1));
  }

  pixels.setPixelColor(0, 0);
  pixels.show();

}


void updateLeds(int sensorValue) {
  // Check if the sensor value is greater than 10
  if (sensorValue > 10) {
    rainbowIndex++;
    if (rainbowIndex >= 7) {
      rainbowIndex = 0;
    }
    for (int i = 0; i < MIN_LED_COUNT; i++) {
      float per=0.05+(0.95/MIN_LED_COUNT)*i;
      pixels.setPixelColor(i, pixels.Color(rainbowColors[rainbowIndex][0] * per, rainbowColors[rainbowIndex][1] * per, rainbowColors[rainbowIndex][2] * per));
    }
    pixels.show();
  }
}