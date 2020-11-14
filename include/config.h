
//General
#define VERBOSE               (1)                // set 1 to enable verbose serial and perform debug

#define VERSION              (1)                // firmware version
#define BAUD                 (9600)             // serial boud rate
#define MAX_BUF              (64)               // buffer size


// Steppers and hardware set up 
#define STEPS_PER_TURN_X     (2038)
#define STEPS_PER_TURN_Y     (20)
#define STEPS_PER_TURN_Z     (2038)
#define STEPS_PER_TURN_E     (20)  

#define PITCH_X     (40)                         // Pitch in mm per turn Gt3 40mm per turn
#define PITCH_Y     (0.5)
#define PITCH_Z     (40)
#define PITCH_E     (0.5)  

#define MICRO_STEPPING_X     (1)               // e.g. 4 for 1/4-step
#define MICRO_STEPPING_Y     (1)
#define MICRO_STEPPING_Z     (1)
#define MICRO_STEPPING_E     (1)  

#define NUM_AXIES            (4)
#define MODE                 (0)                //0: 2 axis mirror 1: 2+2 axis independent 

//Stepper 28BYJ-48, the feed rate should be within 50 and 500 steps per second

//PIN mapping CNC_Shield v3.xx
#define ENABLE                  (8)             //same for all steppers

#define X_STEP                  (2)
#define X_DIR                   (5)
#define X_LIMIT                 (9)

#define Y_STEP                  (3)
#define Y_DIR                   (6)
#define Y_LIMIT                 (10)

#define Z_STEP                  (4)
#define Z_DIR                   (7)
#define Z_LIMIT                 (11)

#define E_STEP                  (12)            //mapping spindle enable
#define E_DIR                   (13)            //mapping spindle direction
#define E_LIMIT                 (17)            //fix this, Z and E have same limit switch