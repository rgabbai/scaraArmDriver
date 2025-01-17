/* Define single-letter commands that will be sent over the serial link.*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define GET_BAUDRATE   'b'
#define JOY_MODE_Z     'z'
#define JOY_MODE_XY    'j'
#define INV_KINEMATIC  'i'      // arg1 = x arg2 =y -> result in XY movment based on inverse kinematic 
#define RESET_MOTOR    'r'      // arg1 = 0,1,2 (corrospond to x,y,z) arg = 3 : reset all x,y & z together
#define PAUSE          'p'      // Stop all operations/modes
#define SAVE_PATH_XY   's'      // save path
#define LOAD_PATH_XY   'l'      // load path and replay arg1 # of times
#define ARM_LOCATION   'a'      // retrive th1 & th2 angle locations


#endif
