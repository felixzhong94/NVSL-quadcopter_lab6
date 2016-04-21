#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include "Alarms.h"

char tmpbuf[128];

/**************************************************************************************/
/***************                  Motor Pin order                  ********************/
/**************************************************************************************/
// since we are uing the PWM generation in a direct way, the pin order is just to inizialie the right pins 
// its not possible to change a PWM output pin just by changing the order
uint8_t PWM_PIN[8] = {8,3,5,4,6,5,A2,12};   //for a quad+: rear,right,left,front

/**************************************************************************************/
/************  Writes the Motors values to the PWM compare register  ******************/
/**************************************************************************************/
void writeMotors() { // [1000;2000] => [125;250]

  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI)	//Values where modified to drive brushed. Values go from 0 to 250
    #if (NUMBER_MOTOR > 0)
        OCR1A = (motor[0] <= MINTHROTTLE)? 0 : (int16_t) ((motor[0] - 1000)+8); // From 0...1008,  pin 8
    #endif
    #if (NUMBER_MOTOR > 1)
        OCR3A = (motor[1] <= MINTHROTTLE)? 0 : (int16_t) ((motor[1] - 1000)+8); // From 0...1008,  pin 3
    #endif
    #if (NUMBER_MOTOR > 2)
        OCR3C = (motor[2] <= MINTHROTTLE)? 0 : (int16_t) ((motor[2] - 1000)+8); // From 0...1008, pin 5
        //(Note: This is the only timer that is 8 bits)
        //OCR2A = (motor[2] - 1000)>>2; //  pin 9
    #endif
    #if (NUMBER_MOTOR > 3)
        OCR3B = (motor[3] <= MINTHROTTLE)? 0 : (int16_t) ((motor[3] - 1000)+8); // From 0...1008, pin 4
    #endif
  #endif
}

/**************************************************************************************/
/************          Writes the mincommand to all Motors           ******************/
/**************************************************************************************/
void writeAllMotors(int16_t mc) {   // Sends commands to all motors
  for (uint8_t i =0;i<NUMBER_MOTOR;i++) {
    motor[i]=mc;
  }
}

/**************************************************************************************/
/************        Initialize the PWM Timers and Registers         ******************/
/**************************************************************************************/
void initOutput() {
  /****************            mark all PWM pins as Output             ******************/
  for(uint8_t i=0;i<NUMBER_MOTOR;i++) {
    pinMode(PWM_PIN[i],OUTPUT);
  }
  
  /********  Specific PWM Timers & Registers for the atmega328P (Promini)   ************/
  #if defined(PROMINI) || defined(ATMEGA128RF)
    #if (NUMBER_MOTOR > 0)
      TCCR1A |= _BV(COM1A1); // connect pin 8 to timer (OCR1A) channel A
    #endif
    #if (NUMBER_MOTOR > 1)
      TCCR3A |= _BV(COM3A1); // connect pin 3 to timer (OCR3A) channel A
    #endif
    #if (NUMBER_MOTOR > 2)
      TCCR3A |= _BV(COM3C1); // connect pin 5 to timer (OCR3C) channel C
      //TCCR2A |= _BV(COM2A1); // connect pin 9 to timer (OCR2A) channel A
    #endif
    #if (NUMBER_MOTOR > 3)
      TCCR3A |= _BV(COM3B1); // connect pin 4 to timer (OCR3B) channel BS
    #endif

    ICR1   |= 0x03FF; // TOP to 1023;
    ICR3   |= 0x03FF; // TOP to 1023;

    //Timer 1 to 10 bits correct pwm phase(0xA) no prescaler
	TCCR1A &= ~(1<<WGM30);
	TCCR1A |= (1<<WGM31);
	TCCR1B &= ~(1<<WGM32) &  ~(1<<CS11) & ~(1<<CS12);
	TCCR1B |= (1<<WGM33) | (1<<CS10);

    //Timer 3 to 10 bits correct pwm phasr (0x3)
	TCCR3A &= ~(1<<WGM30);
	TCCR3A |= (1<<WGM31);
	TCCR3B &= ~(1<<WGM32) &  ~(1<<CS11) & ~(1<<CS12);
	TCCR3B |= (1<<WGM33) | (1<<CS10);

  #endif
  
  writeAllMotors(MINCOMMAND);
  delay(300);
}


/**************************************************************************************/
/********** Mixes the Computed stabilize values to the Motors & Servos  ***************/
/**************************************************************************************/
void mixTable() {
  int16_t maxMotor;
  uint8_t i;
  #define PIDMIX(X,Y,Z) rcCommand[THROTTLE] + axisPID[ROLL]*X + axisPID[PITCH]*Y + YAW_DIRECTION * axisPID[YAW]*Z

  /****************                   main Mix Table                ******************/
  #if defined( MY_PRIVATE_MIXING )
    #include MY_PRIVATE_MIXING
  #elif defined( BI )
    motor[0] = PIDMIX(+1, 0, 0); //LEFT
    motor[1] = PIDMIX(-1, 0, 0); //RIGHT
    servo[4] = (SERVODIR(4,2) * axisPID[YAW]) + (SERVODIR(4,1) * axisPID[PITCH]) + get_middle(4); //LEFT
    servo[5] = (SERVODIR(5,2) * axisPID[YAW]) + (SERVODIR(5,1) * axisPID[PITCH]) + get_middle(5); //RIGHT
  #elif defined( TRI )
    motor[0] = PIDMIX( 0,+4/3, 0); //REAR
    motor[1] = PIDMIX(-1,-2/3, 0); //RIGHT
    motor[2] = PIDMIX(+1,-2/3, 0); //LEFT
    servo[5] = (SERVODIR(5, 1) * axisPID[YAW]) + get_middle(5); //REAR
  #elif defined( QUADP )
    motor[0] = PIDMIX( 0,+1,-1); //REAR
    motor[1] = PIDMIX(-1, 0,+1); //RIGHT
    motor[2] = PIDMIX(+1, 0,+1); //LEFT
    motor[3] = PIDMIX( 0,-1,-1); //FRONT
  #elif defined( QUADX )
    motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    motor[3] = PIDMIX(+1,-1,-1); //FRONT_L
  #else
    #error "missing coptertype mixtable entry. Either you forgot to define a copter type or the mixing table is lacking neccessary code"
  #endif // MY_PRIVATE_MIXING

  /****************                normalize the Motors values                ******************/
    maxMotor=motor[0];
    for(i=1; i< NUMBER_MOTOR; i++)
      if (motor[i]>maxMotor) maxMotor=motor[i];
    for(i=0; i< NUMBER_MOTOR; i++) {
      if (maxMotor > MAXTHROTTLE) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        motor[i] -= maxMotor - MAXTHROTTLE;
      motor[i] = constrain(motor[i], conf.minthrottle, MAXTHROTTLE);
      if ((rcData[THROTTLE] < MINCHECK) && !f.BARO_MODE)
        motor[i] = conf.minthrottle;
      if (!f.ARMED)
        motor[i] = MINCOMMAND;
    }
}
