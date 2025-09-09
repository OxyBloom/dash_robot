/* *************************************************************
   Encoder definitions
   
   Add an "#ifdef" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
    
  ISR (PCINT0_vect){
  	static uint8_t left_enc_last = 0;
    static uint8_t right_enc_last = 0;
        
	// Left encoder: read PB6 and PB7
  left_enc_last <<= 2; // Shift previous state two bits
  left_enc_last |= (PINB & ((1 << PB6) | (1 << PB7))) >> PB6; // Read PB6 and PB7
  left_enc_pos += ENC_STATES[left_enc_last & 0x0F];

  // Right encoder: read PB4 and PB5
  right_enc_last <<= 2; // Shift previous state two bits
  right_enc_last |= (PINB & ((1 << PB4) | (1 << PB5))) >> PB4; // Read PB4 and PB5
  right_enc_pos += ENC_STATES[right_enc_last & 0x0F];
  }
  
  // Encoder Interrupt Service Routine for Right Motor
  // volatile unsigned long lastInterruptTimeRight = 0;
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

  // void rightEncoderCallback() {
  //   unsigned long interruptTime = millis(); // Current time in ms
  //   if (interruptTime - lastInterruptTimeRight > 2) { // Debounce: Ignore interrupts within 2 ms
  //     if (digitalRead(RIGHT_ENC_PIN_B) == HIGH) {
  //       // right_encoder_sign = "p"; // Positive direction
  //       right_enc_pos++;  // Increment count
  //     } else {
  //       // right_encoder_sign = "n"; // Negative direction
  //       right_enc_pos--;  // Decrement count
  //     }
  //     lastInterruptTimeRight = interruptTime;
  //   }
  // }

  // // Encoder Interrupt Service Routine for Left Motor
  // volatile unsigned long lastInterruptTimeLeft = 0;

  // void leftEncoderCallback() {
  //   unsigned long interruptTime = millis(); // Current time in ms
  //   if (interruptTime - lastInterruptTimeLeft > 2) { // Debounce: Ignore interrupts within 2 ms
  //     if (digitalRead(LEFT_ENC_PIN_B) == HIGH) {
  //       // left_encoder_sign = "p";  // Positive direction
  //       left_enc_pos++;   // Increment count
  //     } else {
  //       // left_encoder_sign = "n";  // Negative direction
  //       left_enc_pos--;   // Decrement count
  //     }
  //     lastInterruptTimeLeft = interruptTime;
  //   }
  // }


#else
  // #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
// void resetEncoders() {
//   resetEncoder(LEFT);
//   resetEncoder(RIGHT);
// }

#endif

