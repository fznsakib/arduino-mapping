#define L_PWM_PIN  10
#define L_DIR_PIN  16
#define R_PWM_PIN  9
#define R_DIR_PIN 15

#define FORWARD   LOW
#define BACKWARD  HIGH

#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1

void setup_motor_pins() {
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  digitalWrite( L_DIR_PIN, FORWARD );
  digitalWrite( R_DIR_PIN, FORWARD );
}

void drive_motor(int wheel, float speed) {
  int dir_pin, pwm_pin;
  if (wheel == LEFT_MOTOR) {
    dir_pin = L_DIR_PIN;
    pwm_pin = L_PWM_PIN;
  } else {
    dir_pin = R_DIR_PIN;
    pwm_pin = R_PWM_PIN;
  }
  if (speed < 0) {
    speed = -speed;
    digitalWrite(dir_pin, BACKWARD);
  } else {
    digitalWrite(dir_pin, FORWARD);
  }
  if (speed > 255) speed = 255;
  analogWrite(pwm_pin, speed);
}