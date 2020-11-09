//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)       // add to get a lot more serial output.

#define VERSION              (1)                      // firmware version
#define BAUD                 (9600)                 // How fast is the Arduino talking?(BAUD Rate of Arduino)
#define MAX_BUF              (64)                     // What is the longest message Arduino can store?

#define STEPS_PER_TURN_X     (20)
#define STEPS_PER_TURN_Y     (4047)
#define STEPS_PER_TURN_Z     (20)
#define STEPS_PER_TURN_E     (4047)  

#define PITCH_X     (2)                             // Pitch in turns per mm
#define PITCH_Y     (2)
#define PITCH_Z     (2)
#define PITCH_E     (2)  

#define MICRO_STEPPING_X     (16)                    // e.g. 4 for 1/4-step
#define MICRO_STEPPING_Y     (1)
#define MICRO_STEPPING_Z     (1)
#define MICRO_STEPPING_E     (1)  

#define NUM_AXIES            (4)
#define MODE                 (0)                    //0: 2 axis mirror 1: 2+2 axis independent 