int ledPin = 13;       // Pin where the LED is connected
bool ledState = LOW;   // Initial LED state
char receivedChar = '1'; // Default state (to start with)
unsigned long previousMillis = 0; // Stores the last time the LED was updated
const long interval = 500;   // Interval for LED blink (in milliseconds)
bool shouldBlink = true;    // Determines if the LED should blink or stay solid

void setup() {
  pinMode(ledPin, OUTPUT);   // Set the LED pin as an output
  Serial.begin(9600);        // Initialize serial communication at 9600 baud rate
}

void loop() {
  // Check for new serial data
  if (Serial.available() > 0) {
    receivedChar = Serial.read(); // Read the incoming serial data
    // Set the blinking state based on the received character
    if (receivedChar == '1') {
      shouldBlink = true; // Enable blinking mode
    } else if (receivedChar == '0') {
      shouldBlink = false; // Disable blinking, keep solid on
      digitalWrite(ledPin, HIGH); // Turn the LED on solid immediately
    }
  }

  // Get the current time
  unsigned long currentMillis = millis();

  // Handle blinking behavior
  if (shouldBlink) {
    // Check if it's time to toggle the LED
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;   // Save the last time the LED was toggled
      ledState = !ledState;             // Toggle LED state
      digitalWrite(ledPin, ledState);   // Write the new state to the LED
    }
  }
}
