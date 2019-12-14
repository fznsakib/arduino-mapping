#define RIGHT_ENCODER_A_PIN  7
#define RIGHT_ENCODER_B_PIN  23
#define LEFT_ENCODER_A_PIN   26

// Volatile Global variables used by Encoder ISR.
volatile long right_encoder_count; // used by encoder to count the rotation
volatile bool right_old_A;  // used by encoder to remember prior state of A
volatile bool right_old_B;  // used by encoder to remember prior state of B

volatile long left_encoder_count; // used by encoder to count the rotation
volatile bool left_old_A;  // used by encoder to remember prior state of A
volatile bool left_old_B;  // used by encoder to remember prior state of B

// Global volatile timestamps to determine
// speed from within the left ISR.
volatile unsigned long left_interval;
volatile unsigned long left_last_time;
volatile unsigned long left_now;

// Global volatile timestamps to determine
// speed from within the right ISR.
volatile unsigned long right_interval;
volatile unsigned long right_last_time;
volatile unsigned long right_now;

volatile long previous_left_encoder_count;
volatile long previous_right_encoder_count;

volatile float timer_left_speed;
volatile float timer_right_speed;

volatile int hertz;

// This ISR handles just Encoder 1 (Right Encoder)
// ISR to read the Encoder1 Channel A and B pins
// and then look up based on transition what kind of
// rotation must have occured.
ISR( INT6_vect ) {

  right_now = micros();
  right_interval = right_now - right_last_time;
  right_last_time = right_now;

  // First, Read in the new state of the encoder pins.
  // Standard pins, so standard read functions.
  boolean right_new_B = digitalRead( RIGHT_ENCODER_B_PIN );
  boolean right_new_A = digitalRead( RIGHT_ENCODER_A_PIN );

  // Some clever electronics combines the
  // signals and this XOR restores the
  // true value.
  right_new_A ^= right_new_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:

  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;
  state = state | ( right_new_A  << 3 );
  state = state | ( right_new_B  << 2 );
  state = state | ( right_old_A  << 1 );
  state = state | ( right_old_B  << 0 );

  // This is an inefficient way of determining
  // the direction. However it illustrates well
  // against the lecture slides.
  switch ( state ) {
    case 1:  right_encoder_count--; break;  // clockwise?
    case 2:  right_encoder_count++; break;  // anti-clockwise?
    case 4:  right_encoder_count++; break;  // anti-clockwise?
    case 7:  right_encoder_count--; break;  // clockwise?
    case 8:  right_encoder_count--; break;  // clockwise?
    case 11: right_encoder_count++; break;  // anti-clockwise?
    case 13: right_encoder_count++; break;  // anti-clockwise?
    case 14: right_encoder_count--; break;  // clockwise?
  }

  // Save current state as old state for next call.
  right_old_A = right_new_A;
  right_old_B = right_new_B;

}

// This ISR handles just Encoder 0 (Left Encoder)
// ISR to read the Encoder0 Channel A and B pins
// and then look up based on transition what kind of
// rotation must have occured.
ISR( PCINT0_vect ) {

  left_now = micros();
  left_interval = left_now - left_last_time;
  left_last_time = left_now;

  // First, Read in the new state of the encoder pins.

  // Mask for a specific pin from the port.
  // Non-standard pin, so we access the register
  // directly.  
  // Reading just PINE would give us a number
  // composed of all 8 bits.  We want only bit 2.
  // B00000100 masks out all but bit 2
  boolean left_new_B = PINE & (1<<PINE2);
  //boolean newE0_B = PINE & B00000100;  // Does same as above.

  // Standard read for the other pin.
  boolean left_new_A = digitalRead( LEFT_ENCODER_A_PIN ); // 26 the same as A8

  // Some clever electronics combines the
  // signals and this XOR restores the 
  // true value.
  left_new_A ^= left_new_B;

  // Create a bitwise representation of our states
  // We do this by shifting the boolean value up by
  // the appropriate number of bits, as per our table
  // header:

  // State :  (bit3)  (bit2)  (bit1)  (bit0)
  // State :  New A,  New B,  Old A,  Old B.
  byte state = 0;                   
  state = state | ( left_new_A  << 3 );
  state = state | ( left_new_B  << 2 );
  state = state | ( left_old_A  << 1 );
  state = state | ( left_old_B  << 0 );

  // This is an inefficient way of determining
  // the direction.  However it illustrates well
  // against the lecture slides.  
  switch( state ) {
    case 1:  left_encoder_count--; break;  // clockwise?
    case 2:  left_encoder_count++; break;  // anti-clockwise?
    case 4:  left_encoder_count++; break;  // anti-clockwise?
    case 7:  left_encoder_count--; break;  // clockwise?
    case 8:  left_encoder_count--; break;  // clockwise?
    case 11: left_encoder_count++; break;  // anti-clockwise?
    case 13: left_encoder_count++; break;  // anti-clockwise?
    case 14: left_encoder_count--; break;  // clockwise?
  }

  // Save current state as old state for next call.
  left_old_A = left_new_A;
  left_old_B = left_new_B; 
}


// This setup routine enables interrupts for
// encoder1. The interrupt is automatically
// triggered when one of the encoder pin changes.
// This is really convenient! It means we don't
// have to check the encoder manually.
void setup_right_encoder() {

  // Initialise our count value to 0.
  right_encoder_count = 0;

  // Initialise the prior A & B signals
  // to zero, we don't know what they were.
  right_old_A = 0;
  right_old_B = 0;

  // Setup pins for encoder 1
  pinMode( RIGHT_ENCODER_A_PIN, INPUT );
  pinMode( RIGHT_ENCODER_B_PIN, INPUT );

  // Now to set up PE6 as an external interupt (INT6), which means it can
  // have its own dedicated ISR vector INT6_vector

  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit low, preserve other bits
  EIMSK = EIMSK & ~(1<<INT6);
  //EIMSK = EIMSK & B1011111; // Same as above.
  
  // Page 89, 11.1.2 External Interrupt Control Register B – EICRB
  // Used to set up INT6 interrupt
  EICRB |= ( 1 << ISC60 );  // using header file names, push 1 to bit ISC60
  //EICRB |= B00010000; // does same as above

  // Page 90, 11.1.4 External Interrupt Flag Register – EIFR
  // Setting a 1 in bit 6 (INTF6) clears the interrupt flag.
  EIFR |= ( 1 << INTF6 );
  //EIFR |= B01000000;  // same as above

  // Now that we have set INT6 interrupt up, we can enable
  // the interrupt to happen
  // Page 90, 11.1.3 External Interrupt Mask Register – EIMSK
  // Disable external interrupts for INT6 first
  // Set INT6 bit high, preserve other bits
  EIMSK |= ( 1 << INT6 );
  //EIMSK |= B01000000; // Same as above

}

void setup_left_encoder() {

    // Initialise our count value to 0.
    left_encoder_count = 0;

    // Initialise the prior A & B signals
    // to zero, we don't know what they were.
    left_old_A = 0;
    left_old_B = 0;

    // Setting up E0_PIN_B:
    // The Romi board uses the pin PE2 (port E, pin 2) which is
    // very unconventional. It doesn't have a standard
    // arduino alias (like d6, or a5, for example).
    // We set it up here with direct register access
    // Writing a 0 to a DDR sets as input
    // DDRE = Data Direction Register (Port)E
    // We want pin PE2, which means bit 2 (counting from 0)
    // PE Register bits [ 7  6  5  4  3  2  1  0 ]
    // Binary mask      [ 1  1  1  1  1  0  1  1 ]

    // By performing an & here, the 0 sets low, all 1's preserve
    // any previous state.
    DDRE = DDRE & ~(1<<DDE6);
    //DDRE = DDRE & B11111011; // Same as above. 

    // We need to enable the pull up resistor for the pin
    // To do this, once a pin is set to input (as above)
    // You write a 1 to the bit in the output register
    PORTE = PORTE | (1<< PORTE2 );
    //PORTE = PORTE | 0B00000100;

    // Encoder0 uses conventional pin 26
    pinMode( LEFT_ENCODER_A_PIN, INPUT );
    digitalWrite( LEFT_ENCODER_A_PIN, HIGH ); // Encoder 0 xor

    // Enable pin-change interrupt on A8 (PB4) for encoder0, and disable other
    // pin-change interrupts.
    // Note, this register will normally create an interrupt a change to any pins
    // on the port, but we use PCMSK0 to set it only for PCINT4 which is A8 (PB4)
    // When we set these registers, the compiler will now look for a routine called
    // ISR( PCINT0_vect ) when it detects a change on the pin. PCINT0 seems like a
    // mismatch to PCINT4, however there is only the one vector servicing a change
    // to all PCINT0->7 pins.
    // See Manual 11.1.5 Pin Change Interrupt Control Register - PCICR
    
    // Page 91, 11.1.5, Pin Change Interrupt Control Register 
    // Disable interrupt first
    PCICR = PCICR & ~( 1 << PCIE0 );
    // PCICR &= B11111110;  // Same as above
    
    // 11.1.7 Pin Change Mask Register 0 – PCMSK0
    PCMSK0 |= (1 << PCINT4);
    
    // Page 91, 11.1.6 Pin Change Interrupt Flag Register – PCIFR
    PCIFR |= (1 << PCIF0);  // Clear its interrupt flag by writing a 1.

    // Enable
    PCICR |= (1 << PCIE0);
}

// The ISR routine.
// The name TIMER3_COMPA_vect is a special flag to the 
// compiler.  It automatically associates with Timer3 in
// CTC mode.
ISR( TIMER3_COMPA_vect ) {
  long left_diff = left_encoder_count - previous_left_encoder_count;
  previous_left_encoder_count = left_encoder_count;
  timer_left_speed = 7.f * (float) left_diff / (float) hertz;

  long right_diff = right_encoder_count - previous_right_encoder_count;
  previous_right_encoder_count = right_encoder_count;
  timer_right_speed = 7.f * (float) right_diff / (float) hertz;
}

// Routine to setup timer3 with a specified frequency.
void setup_timer3( int hz ) {

  hertz = hz;

  // disable global interrupts
  cli();          

  // Reset timer3 to a blank condition.
  // TCCR = Timer/Counter Control Register
  TCCR3A = 0;     // set entire TCCR3A register to 0
  TCCR3B = 0;     // set entire TCCR3B register to 0
  
  // First, turn on CTC mode.  Timer3 will count up
  // and create an interrupt on a match to a value.
  // See table 14.4 in manual, it is mode 4.
  TCCR3B = TCCR3B | (1 << WGM32);

  int prescalers[5] = {1024, 256, 64, 8, 1};
  int prescaler = -1;
  int CMC = -1;

  for (int i = 0; i < 5; i++) {
    float result = 16000000.f / (float) prescalers[i] / (float) hertz;

    if (floor(result) == result && result < 65536) {
      prescaler = prescalers[i];
      CMC = (int) result;
    }
  }

  // Set prescaler value.
  switch (prescaler) {
    case (1):
      TCCR3B = TCCR3B | (1 << CS30);
      break;
    case (8):
      TCCR3B = TCCR3B | (1 << CS31);
      break;
    case (64):
      TCCR3B = TCCR3B | (1 << CS30);
      TCCR3B = TCCR3B | (1 << CS31);
      break;
    case (256):
      TCCR3B = TCCR3B | (1 << CS32);
      break;
    case (1024):
      TCCR3B = TCCR3B | (1 << CS30);
      TCCR3B = TCCR3B | (1 << CS32);
      break;
    default:
      // Couldn't find one. Exit or fail or something!
      break;
  }

  // Set Compare Match counter value
  OCR3A = CMC;

  // enable timer compare interrupt:
  TIMSK3 = TIMSK3 | (1 << OCIE3A);

  // enable global interrupts:
  sei();
}

