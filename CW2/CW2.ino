#include "CW2.h"

#define STATE_INITIAL            0
#define STATE_TURN_TO_THETA      1
#define STATE_PRINT_DISTANCES    2

int STATE;

void setup() {

  setup_led_pins();
  setup_motor_pins();
  setup_left_encoder();
  setup_right_encoder();
  setup_timer3(100);
  calibrate_sensors();
  pinMode( BUTTON_B, INPUT );
  digitalWrite( BUTTON_B, HIGH );

  initialise_main_variables();
  initialise_timer_variables();

  Serial.begin(9600);

  STATE = STATE_INITIAL;

}


void loop() {

  binary_led(STATE);

  kinematics.update(left_encoder_count, right_encoder_count);

  // Serial.print(kinematics.get_theta());

  switch(STATE) {

    case STATE_INITIAL:
      initialising_beeps();
      break;

    case STATE_TURN_TO_THETA:
      turn_to_theta();
      break;

    case STATE_PRINT_DISTANCES:
      print_distances();
      break;

    default:
      // Should never reach here!
      break;
  }

  // Small delay. I've never used this but maybe use it??
  // delay(10);
}


void initialising_beeps() {

  initial_beeps_time_now = millis();

  if (initial_beeps <= 2) {
    initial_beeps_time_elapsed = initial_beeps_time_now - initial_beeps_timestamp;
    if (initial_beeps_time_elapsed < 500) {
      if (initial_beep) {
        analogWrite(BUZZER_PIN, 10);
      } else {
        analogWrite(BUZZER_PIN,  0);
      }
    } else {
      initial_beeps_timestamp = initial_beeps_time_now;
      initial_beep = !initial_beep;
      initial_beeps += 1;
    }

  } else  {

    analogWrite(BUZZER_PIN,  0);
    left_speed_PID.reset();
    right_speed_PID.reset();
    approach_pid_timestamp = millis();

    STATE = STATE_TURN_TO_THETA;

  }
}

void turn_to_theta() {

  home_theta = 2 * M_PI;
  difference = home_theta - kinematics.get_theta();

  if (abs(difference) > 0.005) {

    turn_to_theta_pid_time_now = millis();

    turn_to_theta_pid_time_elapsed = turn_to_theta_pid_time_now - turn_to_theta_pid_timestamp;
    if (turn_to_theta_pid_time_elapsed > 10) {
      turn_to_theta_pid_timestamp = turn_to_theta_pid_time_now;

      left_speed_output = left_speed_PID.update(constrain(difference, -0.07, 0.07), timer_left_speed);
      right_speed_output = right_speed_PID.update(-constrain(difference, -0.07, 0.07), timer_right_speed);

    }

    // drive_motor(LEFT_MOTOR,  left_speed_output);
    // drive_motor(RIGHT_MOTOR, right_speed_output);

    int degrees = floor(kinematics.get_theta() * 180 / M_PI);

    float ir0 = IRSensor0.getDistanceInMM();
    float ir1 = IRSensor1.getDistanceInMM();
    float diff = abs(ir1 - ir0);
    float offset  = 46;  // mm
    float minDist = 100; // mm

    float distance;
    if(diff < 42) {
      // IR0 is too close
      distance = ir1 - offset;
    } else if((ir0 + offset) >= 400) {
      // IR1 is too far
      distance = ir0;
    } else if(ir0 >= ir1) {
      // both sensors too close
      distance = 0;
    } else {
      // both sensors far enough
      distance  = (ir0 + (ir1 - offset)) / 2;
    }

    distances[degrees] += 1;
    distances2[degrees] = (distances[degrees] + distance) / 2;

    Serial.print(ir0);
    Serial.print(",");
    Serial.println(ir1);
    delay(5);

  } else {

    drive_motor(LEFT_MOTOR,  0);
    drive_motor(RIGHT_MOTOR, 0);

    left_speed_PID.reset();
    right_speed_PID.reset();

    left_speed_output = 0;
    right_speed_output = 0;

    STATE = STATE_PRINT_DISTANCES;

  }

}

void print_distances() {

  int mode = -1;

  do {

    if (SERIAL_ACTIVE) Serial.println("Waiting for button B to print distances...");

    // Get input from buttons
    int btn_b = digitalRead( BUTTON_B );

    // Decide if we are going to print
    // or erase the map.
    if ( btn_b == LOW ) {
      mode = 0;

      Serial.print("box_66_one_n = [");
      // Print distances here
      for (int i=0; i < 360; i++){
        if (i != 0) Serial.print(",");
        Serial.print(distances[i]);
      }
      Serial.println("]");

      Serial.print("box_66_two_n = [");
      // Print distances here
      for (int i=0; i < 360; i++){
        if (i != 0) Serial.print(",");
        Serial.print(distances2[i]);
      }
      Serial.println("]");
      Serial.println("Done");
    }

  } while (mode < 0);

  digitalWrite(RED_PIN   , HIGH);

  delay(2000);
  exit(0);

}
