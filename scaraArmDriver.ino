
/********************************************************* 
scaraArmDriver

This code control Scara Arm stepper motors
It is controlled by ROS node 

Interface:

Written by Rony.gabbai@gmail.com
*********************************************************/

/* Serial port baud rate */
#define BAUDRATE     57600


/* Maximum PWM signal */
#define MAX_PWM        255

/* Include definition of serial commands */
#include "commands.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#include "A4988.h"
#include "StepMotorDriver.h"



// Serial comunication variables 
//--------------------------------------------------------------------
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int indx = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;


// modes 
//bool joyZmode = false;
//--------------------------------------------------------------------


/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  indx = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);




    switch(cmd) {
    case GET_BAUDRATE:
        Serial.println(BAUDRATE);
    break;

    case RESET_MOTOR:
        Serial.print("Reset step motor");
        Serial.println(arg1);
        switch(arg1) {
          case 0:
            resetStepMotor('x');
          break;
          case 1:
            resetStepMotor('y');
          break;
          case 2:
            resetStepMotor('z');
          break;
          default:
              Serial.println("Invalid motor selection in RESET_MOTOR Command");
          break;
        }
      break;
      case JOY_MODE_Z:
        Serial.print("Activate JOY- Z mode");
        joy_z_mode();
        //joyZmode = true;
      break;
      case JOY_MODE_XY:
        Serial.print("Activate JOY- XY mode");
        joy_xy_mode();
      break;

      case PAUSE:
        Serial.print("Pause all modes"); 
        //joyZmode = false;   
      break;

    default:
    Serial.println("Invalid Command");
    break;
  }

  // mode activations
  //if (joyZmode) joy_z_mode();



}

/* Read Serial cmd interface*/
int readSerial() {
    // Read the next character
    chr = Serial.read();
    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[indx] = NULL;
      else if (arg == 2) argv2[indx] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[indx] = NULL;
        arg = 2;
        indx = 0;
      }
      //continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[indx] = chr;
        indx++;
      }
      else if (arg == 2) {
        argv2[indx] = chr;
        indx++;
      }
    }

}


/* Setup function--runs once at startup. */
void setup() {
    Serial.begin(BAUDRATE);

    // Init step motor driver
    initMotorDriver(); 


    /* Attach servos if used */
    #ifdef USE_SERVOS
        int i;
        for (i = 0; i < N_SERVOS; i++) {
            servos[i].initServo(
            servoPins[i],
            stepDelay[i],
            servoInitPosition[i]);
        }
    #endif


}

// Main loop

void loop() {
  // Main Serial interface loop - parsing serial commands 
  //---------------------------------------------------------------
  while (Serial.available() > 0) {
    
    readSerial();
    // Read micro limit switchs
    Xlimit = digitalRead(XLimitPin);
    Ylimit = digitalRead(YLimitPin);
    Zlimit = digitalRead(ZLimitPin);



  } //while
  //-------------------------------------------------------------------
// Sweep servos TBD
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}






