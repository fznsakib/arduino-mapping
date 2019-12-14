#ifndef _Line_follow_h
#define _Line_follow_h

// Class to represent a single line sensor
class LineSensor {
  public:

    // Required function.
    LineSensor(int pin);   // Constructor

    // Suggested functions.
    void calibrate();      // Calibrate
    int readRaw();         // Return the uncalibrated value from the sensor
    int readCalibrated();  // Return the calibrated value from the sensor
    
  private:
  
    int pin;
    int calibration;
    
};


// Class Constructor: 
// Sets pin passed in as argument to input
LineSensor::LineSensor(int Line_pin) {
  pin = Line_pin;
  calibration = 0;
  pinMode(pin, INPUT);
}

// Returns unmodified reading.
int LineSensor::readRaw() {
  return analogRead(pin);
}

// Write this function to measure any
// systematic error in your sensor and
// set some bias values.
void LineSensor::calibrate() {
  int samples = 50;
  unsigned long total = 0;
  for (int i = 0; i < samples; i++) {
    total += readRaw();
  }
  calibration = total / samples;
}

// Use the above bias values to return a
// compensated ("corrected") sensor reading.
int LineSensor::readCalibrated() {
  // Write code to return a calibrated reading here
  return constrain(readRaw() - calibration, 0, 1023);
}

#endif
