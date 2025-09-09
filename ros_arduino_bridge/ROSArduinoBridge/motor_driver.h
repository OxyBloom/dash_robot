/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 13
#endif

// #ifdef MXP_MD2_MOTOR_ADAPTER
//   #define MOTOR0_DIRECTION 50
//   #define MOTOR1_DIRECTION  47
//   #define MOTOR0_PWM   7
//   #define MOTOR1_PWM   8
//   #define MOTOR0_ENABLE 52
//   #define MOTOR1_ENABLE 51
// #endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
