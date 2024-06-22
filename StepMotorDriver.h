
// Stepper Motor names 
#define SM_X            'x'
#define SM_Y            'y'
#define SM_Z            'z'

// Stepper Motor defualts 
#define UP_TH           1000   // Joystic Max value TH
#define DOWN_TH         100    // Joystic min value TH
#define MID_TH          520    // Joystic MID TH
#define XSM_SPEED       450    // X SM starting speed [us]
#define YSM_SPEED       450    // Y SM starting speed [us]
#define ZSM_SPEED       700    // Z SM starting speed [us]
#define XJSPEED         430    // Joystick speed [us]
#define YJSPEED         430    // Joystick speed [us]
#define ZJSPEED         430    // Joystick speed [us]
#define MIN_SPEED       390    // MIN DELAY
#define INTERVAL        200    // DELAY
#define MAX_HEIGHT      290    // MAX HEIGHT [mm] from base 
#define MID_STEPS       1000   //3625   // MID location steps from top Z=0
#define DIST_STEP_RATIO 0.04   // [mm/step]
#define MIN_HEIGHT      110    // MIN Height [mm]
#define MINXNsteps      -500   // MIN left steps
#define MINYNsteps      -500   // MIN left steps
#define RP              30      // Repeat move for X/Y





// Stepper Motor X
  const byte  XstepPin      = 2;  //Z.STEP
  const byte  XdirPin       = 5;  //Z.DIR
  const byte  XLimitPin     = 9; //Z Limit

// Stepper Motor Y
  const byte  YstepPin      = 3;  //Z.STEP
  const byte  YdirPin       = 6;  //Z.DIR
  const byte  YLimitPin     = 10; //Z Limit

// Stepper Motor Z
  const byte  ZstepPin      = 4;  //Z.STEP
  const byte  ZdirPin       = 7;  //Z.DIR
  const byte  ZLimitPin     = 11; //Z Limit

  const byte  enable        = 8;  //Shield Enable 

// joystick - TODO move to different h file..
int vrx = A0; //    Abort pin   - use for VRX
int vry = A1; //    Hold pin    - use for VRY
int sw  = A2; ///   Resume pin  - Use for SW 

// Hold JY values 
int vrx_data = 522; // mid js initial value
int vry_data = 522; 
int vrz_data = 522; //Y AXIS + SW (PUSHED) 
int vrspeed_data = 0; //X AXIS + SW - HOlD Speed X axis
bool sw_data  = false;


// Hold Lmit state
int Xlimit   = 1;
int Ylimit   = 1;
int Zlimit   = 1;

int XNsteps = 0;
int YNsteps = 0;
int ZNsteps = 0;        // Num of steps
long height = 0;        // Height from base  im mm.

int xAxis = 0;          // Hold X axis cordinate 
int yAxis = 0;          // Hold Y axis cordinate 
int zAxis = 0;          // Hold Z axis cordinate 
int XSMSpeed = XSM_SPEED;   //X Stepper Motor Speed - delay in micro sec
int YSMSpeed = YSM_SPEED;   //Y Stepper Motor Speed - delay in micro sec
int ZSMSpeed = ZSM_SPEED;   //Z Stepper Motor Speed - delay in micro sec

bool xSpUpdate = false;
bool ySpUpdate = false;
bool zSpUpdate = false;



bool print_once = true; // use to print once

static unsigned long timer = 0;
unsigned long interval     = INTERVAL;

void initMotorDriver();

void resetStepMotor(char smNum); // localize stepper motor using limit switch

void joy_z_mode();  // Joystic control Z motor 

void joy_xy_mode(); // Joystic control X/Y motor 

