// ***LED strip uses GRB instead of RGB***
// teleop (solid)
// auton (flashing)
// no mans land (green)
// quals mode 
// ramp 
// cones area 
// odrive error (red)

#include <FastLED.h>
#define DATA_PIN 6
#define NUM_LEDS 30 // Number of LEDs in the strip
#define BRIGHTNESS 100 // Brightness (0-255)
#define SPEED 5 // Lower = Faster Movement
CRGB leds[NUM_LEDS];

bool ledState = LOW; // Initial LED state
char receivedChar = '1'; // Default state (to start with)
unsigned long previousMillis = 0; // Stores the last time the LED was updated
const long interval = 500; // Interval for LED blink (in milliseconds)
bool shouldBlink = true; // Determines if the LED should blink or stay solid

void setup() {
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
//  pinMode(ledPin, OUTPUT); // Set the LED pin as an output
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

void loop() {
  // Check for new serial data
  if (Serial.available() > 0) {
    receivedChar = Serial.read(); // Read the incoming serial data
    bool shouldExit = false; // Flag to exit the rainbow loop
    
    // Set the blinking state based on the received character
    if (receivedChar == '1') { // Autonomous mode
      shouldBlink = true;
    } else if (receivedChar == '2') { // Teleop
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB::Blue);
      FastLED.show();
    } else if (receivedChar == '3') { 
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB(0,255,0)); //  red
      FastLED.show();
    } else if (receivedChar == '4') {
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB(255,0,0)); // green
      FastLED.show();
    } else if (receivedChar == '5') {
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB(128,0,128)); // actually cyan
      FastLED.show();
    } else if (receivedChar == '6') {
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB::Cyan); // actually magenta
      FastLED.show();
    } else if (receivedChar == '7') {
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB::Yellow);
      FastLED.show();
    } else if (receivedChar == '8') {
      shouldBlink = false;
      fill_solid(leds, NUM_LEDS, CRGB::White);
      FastLED.show();
    } else if (receivedChar == '9') {  
      shouldBlink = false; // Stop blinking
      static uint8_t startIndex = 0; // Tracks color shift
      while (!shouldExit) {
        startIndex++; // Shift colors for movement

        // Update LED colors for the rainbow effect
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV(startIndex + (i * 256 / NUM_LEDS), 255, 255);
        }

        FastLED.show();
        delay(SPEED); // Controls movement speed

        // Check for new serial data
        if (Serial.available() > 0) {
          char newChar = Serial.read();
          if (newChar == '0') {
            receivedChar = newChar; // Update state
            shouldExit = true; // Exit the rainbow loop
          }
        }
      }
    }
  }

  // Handle blinking behavior
  if (shouldBlink) {
    fill_solid(leds, NUM_LEDS, CRGB::Blue);
    FastLED.show();
    delay(500);
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
    delay(500);
  }
}
