#define ORANGE_PIN 13
#define GREEN_PIN  17
#define RED_PIN    30

void setup_led_pins() {
  pinMode(ORANGE_PIN , OUTPUT);
  pinMode(GREEN_PIN  , OUTPUT);
  pinMode(RED_PIN    , OUTPUT);
}

void binary_led(int x) {
  switch (x) {
    case 0:
      digitalWrite(ORANGE_PIN, LOW);
      digitalWrite(GREEN_PIN , HIGH);
      digitalWrite(RED_PIN   , HIGH);
      break;
    case 1:
      digitalWrite(ORANGE_PIN, LOW);
      digitalWrite(GREEN_PIN , LOW);
      digitalWrite(RED_PIN   , HIGH);
      break;
    case 2:
      digitalWrite(ORANGE_PIN, LOW);
      digitalWrite(GREEN_PIN , HIGH);
      digitalWrite(RED_PIN   , LOW);
      break;
    case 3:
      digitalWrite(ORANGE_PIN, LOW);
      digitalWrite(GREEN_PIN , LOW);
      digitalWrite(RED_PIN   , LOW);
      break;
    case 4:
      digitalWrite(ORANGE_PIN, HIGH);
      digitalWrite(GREEN_PIN , HIGH);
      digitalWrite(RED_PIN   , HIGH);
      break;
    case 5:
      digitalWrite(ORANGE_PIN, HIGH);
      digitalWrite(GREEN_PIN , LOW);
      digitalWrite(RED_PIN   , HIGH);
      break;
    case 6:
      digitalWrite(ORANGE_PIN, HIGH);
      digitalWrite(GREEN_PIN , HIGH);
      digitalWrite(RED_PIN   , LOW);
      break;
    case 7:
      digitalWrite(ORANGE_PIN, HIGH);
      digitalWrite(GREEN_PIN , LOW);
      digitalWrite(RED_PIN   , LOW);
      break;
  }
}