#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h> // Include EEPROM library
#include <Adafruit_NeoPixel.h> 

// Pin definitions
const int SW_PIN = A0;
const int X_PIN = A1;
const int Y_PIN = A2;
const int LED_PIN = 8; // Pin connected to the NeoPixel strip
const int LED_COUNT = 64; // Number of NeoPixels

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Initialize NeoPixel strip

// State variables
bool buttonPressed = false;
bool longPress = false;
unsigned long buttonPressStartTime = 0;
const unsigned long longPressTime = 1000; // Long press time threshold in milliseconds
int frequency = 140; // Initial frequency
uint32_t lastColor = 0x000000; // Last color stored in EEPROM
int lastColorIndex = 0; // Current color index (0-19)
unsigned long lastLEDBlinkTime = 0; // Last time the LED state was changed
bool ledState = false; // Current state of the LED (on/off)

// History of colors
const int colorHistorySize = 20; // Number of stored colors

void drawDisplay();
void savetoEEPROM();
void loadFromEEPROM();
void saveColorToEEPROM(int index, uint32_t color);
void loadColorToEEPROM(int index);
void updateLEDColor(uint32_t color); // New function to update LED color
void blinkLED(); // Function to handle LED blinking

void setup() {
    Serial.begin(115200);
    pinMode(SW_PIN, INPUT_PULLUP); // Set button pin as input with pull-up
    // Initialize OLED display
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.display();

    // Initialize NeoPixel strip
    strip.begin();
    strip.setBrightness(255);
    strip.show(); // Initialize all pixels to 'off'

    // Load last color and frequency from EEPROM
    loadFromEEPROM();
    loadColorToEEPROM(lastColorIndex);

    // Draw initial display
    drawDisplay();
    updateLEDColor(lastColor); // Update LED color
}

void loop() {
    // Read the joystick position
    int yValue = analogRead(Y_PIN);
    int xValue = analogRead(X_PIN);
    buttonPressed = digitalRead(SW_PIN) == LOW;

    // Handle frequency adjustment based on joystick Y position
    if (yValue > 700) {
        if (frequency < 512) {
            frequency++; // Increase frequency
            savetoEEPROM();
            drawDisplay(); // Update display
            //Serial.println("Up");
            if(frequency<50)
            {
                delay(200);
            }
            else
            {
                delay(200/(frequency+1));
            }
        }
    } else if (yValue < 300) {
        if (frequency > 0) {
            frequency--; // Decrease frequency
            savetoEEPROM();
            drawDisplay(); // Update display
            //Serial.println("Down");
           if(frequency<50)
            {
                delay(200);
            }
            else
            {
                delay(200/(frequency+1));
            }
        }
    }

        // Handle left and right button for color history
    if (xValue < 300) {
        lastColorIndex = (lastColorIndex - 1 + colorHistorySize) % colorHistorySize; // Previous color
        loadColorToEEPROM(lastColorIndex);
        savetoEEPROM();
        drawDisplay(); // Update display
        updateLEDColor(lastColor);
        //Serial.println("Left");
        delay(300);
    } else if (xValue > 700) {
        lastColorIndex = (lastColorIndex + 1) % colorHistorySize; // Next color
        loadColorToEEPROM(lastColorIndex);
        savetoEEPROM();
        drawDisplay(); // Update display
        updateLEDColor(lastColor);
        //Serial.println("Right");
        delay(300);
    }

    // Handle button press
    if (buttonPressed) {
        if (!longPress) {
            buttonPressStartTime = millis(); // Record the start time
        }

        // Check for long press
        while (digitalRead(SW_PIN) == LOW) { // Keep checking while button is pressed
            if (millis() - buttonPressStartTime >= longPressTime) {
                //Serial.println("LongPress");
                longPress = true; // Long press detected
                lastColor = 0xFFFFFF; // Change color to white
                saveColorToEEPROM(lastColorIndex, lastColor); // Save new color
                drawDisplay(); // Update display
                updateLEDColor(lastColor);
                break; // Exit loop after detecting long press
            }
        }
        
        delay(10); // Debounce delay
    } else {
        // Button released
        if (longPress) {
            longPress = false; // Reset long press flag
        }
    }

    // Change color on button press
    if (buttonPressed && !longPress) {
        lastColor = random(0x1000000); // Get a new random color
        lastColorIndex = (lastColorIndex - 1 + colorHistorySize) % colorHistorySize; // Previous color
        savetoEEPROM();
        saveColorToEEPROM(lastColorIndex, lastColor); // Save new color
        drawDisplay(); // Update display
        updateLEDColor(lastColor);
        //Serial.println("Click");
        delay(200); // Adjust delay as needed
    }

    // Blink the LED based on the current frequency
    blinkLED();
}

void savetoEEPROM() {
    EEPROM.put(0, lastColor); // Load last color from EEPROM
    EEPROM.put(sizeof(lastColor), frequency); // Load last frequency from EEPROM
    EEPROM.put(sizeof(lastColor) + sizeof(frequency), lastColorIndex); // Load last color index from EEPROM
}

void loadFromEEPROM() {
    EEPROM.get(0, lastColor); // Load last color from EEPROM
    EEPROM.get(sizeof(lastColor), frequency); // Load last frequency from EEPROM
    EEPROM.get(sizeof(lastColor) + sizeof(frequency), lastColorIndex); // Load last color index from EEPROM
}

void saveColorToEEPROM(int index, uint32_t color) {
    EEPROM.put((3 + index) * sizeof(uint32_t), color); // Store color in EEPROM at specific index
}

void loadColorToEEPROM(int index) {
    EEPROM.get((3 + index) * sizeof(uint32_t), lastColor);
}

void updateLEDColor(uint32_t color) {
    strip.fill(color, 0, LED_COUNT); // Fill the strip with the specified color
    strip.show(); // Refresh the LED strip
}

void blinkLED() {
    unsigned long currentMillis = millis();
    unsigned long blinkPeriod = 1000 / frequency; // Calculate blink period based on frequency

    // Toggle LED state based on time
    if (currentMillis - lastLEDBlinkTime >= blinkPeriod) {
        lastLEDBlinkTime = currentMillis; // Update last blink time
        ledState = !ledState; // Toggle LED state

        // Update LED strip based on current state
        if (ledState) {
           updateLEDColor(lastColor);
        } else {
           updateLEDColor(0x000000); 
        }
    }
}

void drawDisplay() {
    display.clearDisplay();
    
    // Draw color in the first line
    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 2);
    display.print("#");
    display.print(lastColor, HEX); // Print color in hex format
    
    // Draw frequency in the second line
    display.setCursor(20, 34);
    display.print(frequency);
    display.print("HZ");
    
    display.display(); // Refresh the display only if changed
}
