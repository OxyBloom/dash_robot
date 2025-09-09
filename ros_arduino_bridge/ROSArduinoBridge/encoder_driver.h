/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PB4 //PD2  //pin 12
  #define LEFT_ENC_PIN_B PB5 //PD3  //pin 13
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PB6  //PIN D10 pin A4
  #define RIGHT_ENC_PIN_B PB7 //PIN D11 //PC5   //pin A5

  // #define LEFT_ENC_PIN_A 19 //PD2  //pin 12
  // #define LEFT_ENC_PIN_B 11 //PD3  //pin 13

//   Encoder 0: Quad Encoder 0
// A – myRIO DIO1
// B – myRIO DIO2
// Encoder 1: Quad Encoder 1
// A – myRIO DIO3
// B – myRIO DIO4
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

