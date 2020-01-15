#include "encoders.h"
#include "irproximity.h"
#include "kinematics.h"
#include "leds.h"
#include "line_sensors.h"
#include "motors.h"
#include "math.h"
#include "pid.h"
#include "timers.h"

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

#define BUZZER_PIN 6
#define BUTTON_B   30

#define IR_PROX_PIN0    A0
#define IR_PROX_PIN1    A2
#define LINE_LEFT_PIN   A2
#define LINE_CENTRE_PIN A3
#define LINE_RIGHT_PIN  A4

LineSensor line_left(LINE_LEFT_PIN);
LineSensor line_centre(LINE_CENTRE_PIN);
LineSensor line_right(LINE_RIGHT_PIN);
SharpIR    IRSensor0( IR_PROX_PIN0 );
SharpIR    IRSensor1( IR_PROX_PIN1 );

#define kp_left_speed 90.00
#define ki_left_speed 0.50
#define kd_left_speed 0.00

#define kp_right_speed 90.00
#define ki_right_speed 0.50
#define kd_right_speed 0.00

#define kp_line 0.0012
#define ki_line 0.00
#define kd_line 0.00

#define kp_heading 13.00
#define ki_heading 0.04
#define kd_heading 0.00

PID left_speed_PID(kp_left_speed, ki_left_speed, kd_left_speed);
PID right_speed_PID(kp_right_speed, ki_right_speed, kd_right_speed);

PID line_PID( kp_line, ki_line, kd_line );

PID heading_PID( kp_heading, ki_heading, kd_heading );

Kinematics kinematics;

int initial_beeps;
bool initial_beep;

float approach_speed;
bool initial_line_found;
float left_speed_output;
float right_speed_output;

int found_beeps;
bool found_beep;

int left_reading, centre_reading, right_reading, total_readings;
float left_likelihood, centre_likelihood, right_likelihood;
int line_likelihood;

float turn;
float follow_line_speed;

bool on_line;
int reading;
int line_threshold;

bool general_line_found;

float home_theta;
float difference;
float heading_output;

float distance;
float previous_distance;
float drive_home_turn;

int degrees;
int distances[360];
int distances2[360];

void calibrate_sensors() {
  line_left.calibrate();
  line_centre.calibrate();
  line_right.calibrate();
}

void initialise_main_variables() {
  initial_beeps = 0;
  initial_beep = true;
  initial_line_found = false;
  approach_speed = 0.5;
  found_beeps = 0;
  found_beep = true;
  left_reading = 0;
  centre_reading = 0;
  right_reading = 0;
  total_readings = 0;
  left_likelihood = 0;
  centre_likelihood = 0;
  right_likelihood = 0;
  line_likelihood = 0;
  turn = 0;
  follow_line_speed = 0.20;
  on_line = false;
  reading = 0;
  line_threshold = 0;
  general_line_found = false;
  home_theta = 0;
  difference = 0;
  heading_output = 0;
  distance = 0;
  previous_distance = 0;
  drive_home_turn = 0;
  degrees = 0;
}

void initialising_beeps();
void turn_to_theta();
void print_distances();
