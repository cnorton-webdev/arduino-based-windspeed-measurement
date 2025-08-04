// Global variables
// Using 'volatile' keyword for variables modified within an Interrupt Service Routine (ISR)
volatile unsigned long start_time_micros = 0; // Stores the timestamp (in microseconds) of the last sensor trigger
volatile unsigned long last_interrupt_time = 0; // For software debouncing the sensor signal
volatile boolean sensor_triggered_flag = false; // Flag to signal that a sensor event has occurred

// Configuration constants
const int SENSOR_PIN = 2; // Digital pin connected to the speed sensor.
                          // On Arduino Uno/Nano, pins 2 and 3 are typically used for external interrupts.
                          // IMPORTANT: Adjust this pin number based on your specific Arduino board and wiring.
                          // Ensure it's an interrupt-capable pin for your board.
const int DEBOUNCE_TIME_MS = 200; // Debounce time in milliseconds

// Function to calculate and print speed
void calculateSpeed() {
  unsigned long done_time_micros = micros(); // Get the current time in microseconds

  // Calculate the elapsed time since the last sensor trigger
  unsigned long elapsed_micros = done_time_micros - start_time_micros;

  // Prevent division by zero if elapsed time is zero (e.g., first run or very fast consecutive triggers)
  if (elapsed_micros == 0) {
    // Serial.println("Elapsed time is zero, skipping speed calculation."); // Uncomment for debugging
    return;
  }

  // Convert elapsed microseconds to seconds for calculations
  double elapsed_seconds = (double)elapsed_micros / 1000000.0;

  double rpm = 60.0 / elapsed_seconds; // Revolutions Per Minute

  // Diameter in cm of object with magnet attached to be read by hall effect sensor
  double distance = rpm * 2;

  double speed = distance; // Speed in units per minute (e.g., cm/min)

  // Assuming '0.0006' is a specific conversion factor to convert the speed to Kilometers Per Hour
  double speedKmh = speed * 0.0006; // Speed in Kilometers Per Hour
  double speedMph = speedKmh * 0.62137119; // Speed in Miles Per Hour

  // Print the calculated speed to the Serial Monitor
  Serial.print("Speed: ");
  Serial.print(speedKmh, 2); // Print with 2 decimal places for precision
  Serial.print(" km/h / ");
  Serial.print(speedMph, 2); // Print with 2 decimal places for precision
  Serial.println(" mp/h", 2); // Print with 2 decimal places for precision

  // Update start_time_micros for the next calculation cycle
  start_time_micros = done_time_micros;
}

// Interrupt Service Routine (ISR) for the sensor pin
// ISRs should be as short and fast as possible.
// They MUST NOT use Serial.print(), delay(), or complex blocking operations.
// Instead, we use a flag (sensor_triggered_flag) to signal the loop() function
// that a sensor event has occurred, and the actual calculation/printing is done in loop().
void detectSensorISR() {
  unsigned long current_millis = millis(); // Get current time for debouncing

  // Implement software debouncing: Check if enough time has passed since the last valid interrupt
  if (current_millis - last_interrupt_time > DEBOUNCE_TIME_MS) {
    // If the interrupt is attached to FALLING (as it is in setup()), this check
    // `digitalRead(SENSOR_PIN) == LOW` is technically redundant for triggering the ISR itself,
    // but it confirms the state and ensures the flag is set only on the desired transition.
    if (digitalRead(SENSOR_PIN) == LOW) {
      sensor_triggered_flag = true; // Set the flag to indicate a sensor event
    }
    last_interrupt_time = current_millis; // Update the last valid interrupt time
  }
}

// Arduino setup function - runs once when the sketch starts
void setup() {
  // Initialize serial communication for debugging and output
  Serial.begin(115200); // Standard baud rate for faster communication
  while (!Serial); // Wait for serial port to connect (especially useful for boards like Leonardo/Micro)

  Serial.println("--- Arduino Speed Sensor Setup For Handheld Windspeed Measurement Device ---");
  Serial.print("Configuring Digital Pin ");
  Serial.print(SENSOR_PIN);
  Serial.println(" as input with internal pull-up resistor.");

  // Configure the sensor pin as an input with an internal pull-up resistor.
  // This means the pin will be HIGH by default, and the sensor should pull it LOW when activated.
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  // Attach an interrupt to the sensor pin.
  // digitalPinToInterrupt(SENSOR_PIN) converts the digital pin number to the interrupt number.
  // detectSensorISR is the function to call when the interrupt occurs.
  // FALLING means the interrupt triggers when the pin goes from HIGH to LOW.
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), detectSensorISR, FALLING);

  // Initialize the start time for the first speed calculation
  start_time_micros = micros();
  Serial.println("System ready. Waiting for sensor triggers...");
}

// Arduino loop function - runs repeatedly after setup()
void loop() {
  // Check if the sensor_triggered_flag is set by the ISR
  if (sensor_triggered_flag) {
    // Reset the flag immediately to ensure calculateSpeed is called only once per trigger
    sensor_triggered_flag = false;

    // Call the speed calculation function.
    // This function can safely use Serial.print() because it's called outside the ISR.
    calculateSpeed();
  }
}