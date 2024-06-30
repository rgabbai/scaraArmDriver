
// Stepper Motor names 
#define SM_X            'x'
#define SM_Y            'y'
#define SM_Z            'z'

// Stepper Motor defualts 
#define UP_TH           1000   // Joystic Max value TH
#define DOWN_TH         100    // Joystic min value TH
#define MID_TH          520    // Joystic MID TH
#define XSM_RST_SPEED   400    // X SM starting speed [us]
#define YSM_RST_SPEED   400    // Y SM starting speed [us]
#define ZSM_RST_SPEED   500    // Z SM starting speed [us]
#define XJSPEED         430    // Joystick speed [us]
#define YJSPEED         430    // Joystick speed [us]
#define ZJSPEED         430    // Joystick speed [us]
#define MIN_SPEED       390    // MIN DELAY
#define INTERVAL        200    // DELAY
#define MAX_HEIGHT      290    // MAX HEIGHT [mm] from base 
#define MID_STEPS       1000   //3625   // MID location steps from top Z=0
#define XMID_STEPS      3400   // MID location steps from X limit
#define YMID_STEPS      7500   // MID location steps from Y limit
#define DIST_STEP_RATIO 0.04   // [mm/step]
#define MIN_HEIGHT      110    // MIN Height [mm]
#define MINXNsteps      -500   // MIN left steps
#define MINYNsteps      -500   // MIN left steps
#define JRP             20     // joy   Repeat move for X/Y
#define XY_RATIO        0.532258
#define XMRANGE         180    // +/- 100 [degree]
#define YMRANGE         100    // [degree]
#define YMRANGE_XMOVE   120    // +/- 100 [degree]
#define XPROX_STEP_DEG  32     // Motor X th1[steps/degree]
#define XPROX_DEG_STEP  0.03125// Motor X th1[degree/steps]
#define YDIST_STEP_DEG  51.77104377  // Motor Y [steps/degree]
#define YDIST_DEG_STEP  0.01931581686 //51.76[steps/degree] Motor Y [degree/steps]
#define YXDIST_DEG_STEP 0.01663306452 //60.121212122[steps/degree] Motor Y degree/steps] due X motor move
#define THETA_TH        1             // delta th of arm theta location search 
#define DX              1            // theta movement for go_to_xy


// Stepper Motor X
  const byte  XstepPin      = 2;  //X.STEP
  const byte  XdirPin       = 5;  //X.DIR
  const byte  XLimitPin     = 9;  //X Limit

// Stepper Motor Y
  const byte  YstepPin      = 3;  //Y.STEP
  const byte  YdirPin       = 6;  //Y.DIR
  const byte  YLimitPin     = 10; //Y Limit

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

long   XNsteps = 0;
float  YNsteps = 0;
float  YNsteps_relative = 0;
int    ZNsteps = 0;        // Num of steps
long   height = 0;        // Height from base  im mm.
float  th1X   = 0;
float  th2Y   = 0;
long   dy     = 0;

int xAxis = 0;          // Hold X axis cordinate 
int yAxis = 0;          // Hold Y axis cordinate 
int zAxis = 0;          // Hold Z axis cordinate 
int XSMSpeed = XSM_RST_SPEED;   //X Stepper Motor Speed - delay in micro sec
int YSMSpeed = YSM_RST_SPEED;   //Y Stepper Motor Speed - delay in micro sec
int ZSMSpeed = ZSM_RST_SPEED;   //Z Stepper Motor Speed - delay in micro sec

bool xSpUpdate = false;
bool ySpUpdate = false;
bool zSpUpdate = false;

int buttonState = 0;
const int buttonStateCount = 10;




bool print_once = true; // use to print once

static unsigned long timer = 0;
unsigned long interval     = INTERVAL;

void initMotorDriver();

void resetStepMotor(char smNum); // localize stepper motor using limit switch

void joy_z_mode();  // Joystic control Z motor 

void joy_xy_mode(); // Joystic control X/Y motor 

bool isButtonPressed();    // detect valid long sw press

void x_motor_step();  // X motor move single step
void y_motor_step();  // Y motor move single step
void goto_xy(long x, long y); // goto xy coordinate 
void calculateInverseKinematics(float x, float y, float *theta1, float *theta2); //inv kinematic


