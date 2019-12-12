
/*****************************************************************************/
#include "timer3.h"     // setup/isr for timer3 to calculate wheel speed.
#include "encoders.h"   // setup and isr to manage encoders.
#include "kinematics.h" // calculates x,y,theta from encoders.
#include "motor.h"      // handles power and direction for motors.
#include "pid.h"        // PID implementation.
#include "LineSensor.h" // handles all 3 line sensors as a single class.
#include "mapping.h"    // Used to store and read a metric byte map to EEPROM.
#include "utils.h"      // Used to generate random gaussian numbers.
#include "irproximity.h"// Used for the ir distance sensor.

//#include "imu.h"          // Advanced, work through labsheet if you wish to use.
//#include "magnetometer.h" // Advanced, work through labsheet if you wish to use.

#include <USBCore.h>    // To fix serial print behaviour bug.
u8 USB_SendSpace(u8 ep);
#define SERIAL_ACTIVE (USB_SendSpace(CDC_TX) >= 50)

/*****************************************************************************
    DEFINITIONS (global)
    Note, pins taken from the pin mapping for Romi available online.
*****************************************************************************/


#define M0_DIR          16  // Motor Pins.
#define M0_PWM          10
#define M1_DIR          15
#define M1_PWM          9

#define L_SENSE_L       A4  // Line sensor pins
#define L_SENSE_C       A3
#define L_SENSE_R       A2

#define BUZZER_PIN      6   // To make the annoying beeping

#define IR_PROX_PIN    A0   // IR Sensor

#define DEBUG_LED      13   // Using the orange LED for debugging

#define BUTTON_A       14   // Push button labelled A on board.
#define BUTTON_B       30   // Push button labelled B on board.

// Behaviour parameters
#define LINE_THRESHOLD        450.00
#define STRAIGHT_FWD_SPEED    5.0
#define LINE_FOLLOW_SPEED     4.0
#define IR_DETECTD_THRESHOLD  80   // a close reading in mm (danger)
#define IR_AVOIDED_THRESHOLD  140   // a distant reading in mm (safe)

// Speed controller for motors.
// Using same gains for left and right.
#define SPD_PGAIN     1.0
#define SPD_IGAIN     0.078
#define SPD_DGAIN     0.006

// PID controller gains for heading feedback
#define H_PGAIN   1.8
#define H_IGAIN   0.0001
#define H_DGAIN   0.0


/*****************************************************************************
    CLASS INSTANCES (global)
    Please investigate class tabs for methods.
*****************************************************************************/

LineSensor  LineSensor( L_SENSE_L, L_SENSE_C, L_SENSE_R );  // Class to handle all 3 line sensors.
Motor       L_Motor( M0_PWM, M0_DIR);                       // To set left motor power.
Motor       R_Motor( M1_PWM, M1_DIR);                       // To set right motor power.
PID         L_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, left.
PID         R_PID( SPD_PGAIN, SPD_IGAIN, SPD_DGAIN );       // Speed control, right.
PID         H_PID( H_PGAIN, H_IGAIN, H_DGAIN );             // Position control, angle.
SharpIR     IRSensor0( IR_PROX_PIN );                       // Get distance to objects (incomplete class)
Kinematics  RomiPose;                                       // Not using ICC.
Mapper      Map;                                            // Default: 25x25 grid, 72mm resolution.


/*****************************************************************************
    OTHER GLOBAL VARIABLES

*****************************************************************************/


unsigned long update_t;     // Used for timing/flow control for main loop()
unsigned long behaviour_t;  // Use to track how long a behaviour has run.

float last_angle_diff;

// used by timer3.h to calculate left and right wheel speed.
volatile float l_speed_t3, r_speed_t3;

// Different states(behaviours) the robot
// can be in.
int STATE;
#define STATE_CALIBRATE       0    // calibrates line sensor
#define STATE_ROTATE          1    // rotates 360 degrees
#define STATE_STOP            2    // stops

#define STATE_RANDOM_WALK     3     // Robot will make random turns 6 seconds
#define STATE_DRIVE_STRAIGHT  4     // Robot drives in a straight line 3 seconds
#define STATE_TURN_TO_ZERO    5     // Turns so the robot faces theta = 0
#define STATE_TURN_TO_PIOVER2 6     // Turns so the robot faces theta = PI/2 (90*)
#define STATE_AVOID_OBSTACLE  7

float distances[360];
float current_angle;



/*****************************************************************************
    SETUP RUNS ONCE ON POWER UP.
*****************************************************************************/
void setup() {

  // Misc pin setup not handled by classes.
  pinMode( BUZZER_PIN, OUTPUT );
  pinMode(DEBUG_LED, OUTPUT );
  pinMode(IR_PROX_PIN, INPUT);

  // Push buttons.  Note that, by doing a
  // digital write, the button has a default
  // (not pressed) value of HIGH.
  pinMode( BUTTON_A, INPUT );
  digitalWrite( BUTTON_A, HIGH );
  pinMode( BUTTON_B, INPUT );
  digitalWrite( BUTTON_B, HIGH );


  // Begin tracking encoder changes.
  setupEncoder0();
  setupEncoder1();

  // Using Timer3 to calcuate wheel speed
  // in the background at 100hz.
  setupTimer3();

  // We set the robot to start kinematics
  // in the centre of the map.
  // See mapping.h for MAP_X/Y definitions.
  RomiPose.setPose( MAP_X / 2, MAP_Y / 2, 0 );

  last_angle_diff = 2*PI + 1;
  current_angle = -1;

  // Start up the serial port.
  Serial.begin(9600);

  // Delay to connect properly.
  delay(1000);

  // Beep so we know if it is reseting.
  beep(); beep(); beep();

  // Print a debug, so we can see a reset on monitor.
  if ( SERIAL_ACTIVE ) Serial.println("***RESET***");

  // This function reads buttons A and B, and will
  // block your Romi from finishing Startup up until
  // a button is pressed.
  decideStartUpFromButtons();

  // set Initial State, also resets timestamps
  changeState( STATE_CALIBRATE );

}// end of setup, Ready to go!



/*****************************************************************************
    LOOP
    REQUIRED MAIN CODE, CALLED ITERATIVELY BY ARDUINO AFTER SETUP
*****************************************************************************/
void loop() {
  
  // Always update kinematics
  RomiPose.update( e0_count, e1_count );

  // Runs a behaviour every 100ms, skips otherwise.
  if (  millis() - update_t > 100 ) {
    // Serial.println(IRSensor0.getDistanceRaw());
    // Serial.println(IRSensor0.getDistanceInMM());

    update_t = millis();

    switch ( STATE ) {
      
      case STATE_CALIBRATE:
        calibrateSensors();
        break;

      case STATE_ROTATE:
        rotateFullCircle();
        break;

      case STATE_STOP:
        stopRobot();
        break;

      default: // unknown, this would be an error.
        reportUnknownState();
        break;

    } // End of state machine switch()
  } // End of update_t if()

}// End of Loop()



/*****************************************************************************
    Helper functions.
    These are used to perform some operations which are not a 
    significant part of the Romi behaviour or state machine.
*****************************************************************************/

// Setup helper function to take user input and initiate
// start up mode.
void decideStartUpFromButtons() {

  // Blocks until either button a or b is
  // pressed.
  // You may wish to improve this code to make
  // the user input more robust.
  int mode = -1;
  
  
  do {

    if ( SERIAL_ACTIVE ) Serial.println("Waiting for button A to start map scan...");

    // Get input from buttons
    int btn_a = digitalRead( BUTTON_A );
    int btn_b = digitalRead( BUTTON_B );

    // Decide if we are going to print
    // or erase the map.
    if ( btn_a == LOW ) {
      mode = 0;
    } else if ( btn_b == LOW ) {
      mode = 1;
    }

  } while ( mode < 0 );

  // Acknowledge button press.
  beep();
}

// Note, this blocks the flow/timing
// of your code.  Use sparingly.
void beep() {
  analogWrite(6, 80);
  delay(50);
  analogWrite(6, 0);
  delay(50);
}

void reportUnknownState() {
  if ( SERIAL_ACTIVE ) {
    Serial.print("Unknown state: ");
    Serial.println( STATE );
  }
}


/*****************************************************************************
    BEHAVIOURS
    An assortment of behaviours called variously by state machine from Loop.
*****************************************************************************/

// The state transition behaviour.
// Resets all the PID, and also beeps so we know it happened.
// Every other behaviour calls this when exiting/transitioning.
// Otherwise, you'll likely see integral wind-up in the PID.
void changeState( int which ) {

  // If, for some reason, we ask to change
  // to the state we are already in, we just
  // return with no action.
  if ( which == STATE ) return;


  // Stop motors.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );

  // A short beep if debugging
  //beep();

  // Reset the timestamp to track how
  // long we exist in the next state (behaviour)
  behaviour_t = millis();

  // If we are changing state, we reset update_t
  // to force an elapsed time before behaviour is
  // actioned (important to stop divide by 0 error.
  update_t = millis();

  // Set the new state to the one requested.
  STATE = which;

  // reset PID
  L_PID.reset();
  R_PID.reset();
  H_PID.reset();

  return;
}

void calibrateSensors() {

  // Make sure motors are off so the robot
  // stays still.
  L_Motor.setPower( 0 );
  R_Motor.setPower( 0 );


  // Line sensor.
  LineSensor.calibrate();

  // Other sensors..?

  // After calibrating, we send the robot to
  // its initial state.
  changeState( STATE_ROTATE );
}


void rotateFullCircle() {
  int inc = 1;
  
  if (current_angle >= 355 - inc) {
    changeState(STATE_STOP);
    return;
  }

  float demand_angle = current_angle + inc;

  float factor = 11.0 / 72.0;

  float demand = 0.10;

  float l_speed = l_speed_t3 * factor;
  float r_speed = r_speed_t3 * factor;

  float L_output = L_PID.update(demand, l_speed); 
  float R_output = R_PID.update(-demand, r_speed);

  // Set motor power.
  L_Motor.setPower(L_output);
  R_Motor.setPower(R_output);

  // convert theta to degree
  float theta = RomiPose.theta * (180 / PI);
  float angle_diff = demand_angle - theta;

  Serial.print(theta);
  Serial.print(", ");
  Serial.println(current_angle);

  if (angle_diff < 0){
    Serial.print("angle measured: ");
    Serial.println(current_angle);
    distances[(int) current_angle] = IRSensor0.getDistanceInMM();
    current_angle += inc;
  }
}

  

void stopRobot() {
  int mode = -1;
  
  do {

    if ( SERIAL_ACTIVE ) Serial.println("Waiting for button B to print distances...");

    // Get input from buttons
    int btn_b = digitalRead( BUTTON_B );

    // Decide if we are going to print
    // or erase the map.
    if ( btn_b == LOW ) {
      mode = 0;

      // Print distances here
      for (int i=0; i < 360; i++){
        // Serial.print(i);
        // Serial.print(" : ");
        Serial.println(distances[i]);
      }
    } 

  } while ( mode < 0 );

  // Acknowledge button press.

  beep();
  exit(0);

}


void turnToThetaPIOver2() {

  float demand_angle = PI / 2;

  // https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  // Some crazy atan2 magic.
  // Treats the difference in angle as cartesian x,y components.
  // Cos and Sin are effectively wrapping the values between -1, +1, with a 90 degree phase.
  // So we can pass in values larger/smaller than 0:TWO_PI fine.
  // atan2 returns -PI/+PI, giving us an indication of direction to turn.
  // Between -PI/+PI also means we avoid an extreme demand sent to the heading PID.
  float diff = atan2( sin( ( demand_angle - RomiPose.theta) ), cos( (demand_angle - RomiPose.theta) ) );

  // If we have got the Romi theta to roughly match
  // the demand (by getting the difference to 0(ish)
  // We transition out of this behaviour.
  if ( abs( diff ) < 0.03 ) {

    // This ensures that the PID are reset
    // and sets the new STATE flag.
    changeState( STATE_STOP  );

  } else {    // else, turning behaviour

    // Measurement is the different in angle, demand is 0
    // bearing steers us to minimise difference toward 0
    float bearing = H_PID.update( 0, diff );

    // Append to motor speed control
    float l_pwr = L_PID.update( (0 - bearing), l_speed_t3 );
    float r_pwr = R_PID.update( (0 + bearing), r_speed_t3 );

    // Set motor power.
    L_Motor.setPower(l_pwr);
    R_Motor.setPower(r_pwr);

  } // end of abs(diff)<0.03 if()

}// end of behaviour

