#include <Arduino.h>
#include <config.h>



//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long nsteps;  // number of steps to move
  long absnsteps;
  long over;  // for dx/dy bresenham calculations
  double stepDelay; // delay necessary for motor to implement coordinated move
  double nextMove; //pointer for next step move time
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;

typedef struct 
{
  unsigned long millis;
  unsigned long nextmove;
} DebugArray;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];
unsigned long StepsCount, DebugCounter =0; 
DebugArray debugArray[1000];

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

float px,py,pz,pe;  // position

// settings
char mode_abs=1;  // absolute mode?

long line_number=0;

float fr=0; // feedrate

//movement steps based on machine params
float STEPS_PER_MM_X   =    (float)STEPS_PER_TURN_X/PITCH_X*MICRO_STEPPING_X;
float STEPS_PER_MM_Y   =    (float)STEPS_PER_TURN_Y/PITCH_Y*MICRO_STEPPING_Y;
float STEPS_PER_MM_Z   =    (float)STEPS_PER_TURN_Z/PITCH_Z*MICRO_STEPPING_Z;  
float STEPS_PER_MM_E   =    (float)STEPS_PER_TURN_E/PITCH_E*MICRO_STEPPING_E;  


//------------------------------------------------------------------------------
// METHODS DECLARATION
//------------------------------------------------------------------------------
void motor_setup();
void motor_enable(bool cmd); 
void where();
void help();
void position(float npx,float npy,float npz,float npe);
void ready();
float Time_Move (float D1, float D2, float UserFeed);
double Step_Delay (float steps, float timeMove);
void Do_Move(float newx,float newy,float newz,float newe, float F);
void pause(long ms);
float parseNumber(char code,float val);
void processCommand();
void homing();



//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// IMPLEMENTATION
//------------------------------------------------------------------------------

/**
 * Startup routine
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  // stepper motor initialisation routine
  motor_setup();
  motor_enable(true);
  

  position(0,0,0,0);  // initialise starting position
  help();  // display splash screen
  ready();
}

/**
 * set up the pins for each stepper motor
 */
void motor_setup() {
  //X stepper
  motors[0].step_pin=X_STEP;
  motors[0].dir_pin=X_DIR;
  motors[0].enable_pin=ENABLE;
  motors[0].limit_switch_pin=X_LIMIT;
  //Y stepper
  motors[1].step_pin=Y_STEP;
  motors[1].dir_pin=Y_DIR;
  motors[1].enable_pin=ENABLE;
  motors[1].limit_switch_pin=Y_LIMIT;
  //Z stepper
  motors[2].step_pin=Z_STEP;
  motors[2].dir_pin=Z_DIR;
  motors[2].enable_pin=ENABLE;
  motors[2].limit_switch_pin=Z_LIMIT;
  //E stepper
  motors[3].step_pin=E_STEP;
  motors[3].dir_pin=E_DIR;
  motors[3].enable_pin=ENABLE;
  motors[3].limit_switch_pin=E_LIMIT;
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
    pinMode(motors[i].limit_switch_pin,INPUT_PULLUP);
  }
}


/**
 * enable / disable stepper motors
 */
void motor_enable(bool cmd) {
  int i;
  switch (cmd)
  {
  case true:
    for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
    }
    /* code */
    break;
  
  default:
    for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
    }
    break;
  }
}

/**
 * Set the logical position
 */
void position(float npx,float npy,float npz,float npe) {
  px=npx;
  py=npy;
  pz=npz;
  pe=npe;
}

/**
 * splash screen
 */
void help() {
  Serial.print(F("Bonobo "));
  Serial.println(VERSION);
  Serial.println(F("RC airplanes foam cutter firmware for Arduino"));
  Serial.println(F("Supported commands:"));
  Serial.println(F("G01 [X/Y/Z/E(coords in mm)] [F(feedrate in mm/sec)] - move to new position"));
  //Serial.println(F("G04 P[seconds]; - delay"));
  //Serial.println(F("G90; - absolute mode"));
  //Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/E(coords in mm)] - set new position"));
  Serial.println(F("G28; - homing cycle"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}

/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready to receive a new command.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.println("CFC");  // signal ready to receive input
  Serial.println(F(">"));  // set up user prompt
}









/**
 * Firmware loop
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  
    char c=Serial.read();  // read from serial
    
    #ifdef VERBOSE
    Serial.print(c); //echo
    #endif
    
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if(c=='\n') { // if cmd received
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for human
      processCommand(); // perform command
      ready(); // allow to receive new command
    }
  }
}



/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parseNumber('G',-1);
  switch(cmd) {
  case  1: { // line
        Do_Move ( 
          parseNumber('X',px),
          parseNumber('Y',py),
          parseNumber('Z',pz),
          parseNumber('E',pe),
          parseNumber('F',fr) 
          );      
    break;
    }
    
  case  2:
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 28:  homing(); break;
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Y',0),
              parseNumber('Z',0),
              parseNumber('E',0) );
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case  17:  motor_enable(true);  break;
  case  18:  motor_enable(false);  break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}

/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=buffer;  // start at the beginning of buffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}

void MotorStep (int MotorId) {
  digitalWrite(motors[MotorId].step_pin,HIGH);
  digitalWrite(motors[MotorId].step_pin,LOW);
  StepsCount++; // debug
}

/**
 * Drives motor to implement move
 */

void Do_Move(float newx,float newy,float newz,float newe, float F) {



  a[0].nsteps = abs(newx-px)*STEPS_PER_MM_X;
  a[1].nsteps = abs(newy-py)*STEPS_PER_MM_Y;

   Serial.println("STEPS_PER_MM_X");
  Serial.println(STEPS_PER_MM_X);

  if (MODE ==1) { // defines move in case of 2+2 axis or 4 independent axis
    a[2].nsteps = abs(newz-pz)*STEPS_PER_MM_Z;
    a[3].nsteps = abs(newe-pe)*STEPS_PER_MM_E;
  } else {
    a[2].nsteps = abs(newx-px)*STEPS_PER_MM_Z;
    a[3].nsteps = abs(newy-py)*STEPS_PER_MM_E;
  }

  if (a[0].nsteps ==0 && a[1].nsteps ==0 && a[2].nsteps ==0 && a[3].nsteps ==0) { return;}; // exit if no move required
  


  digitalWrite(motors[0].dir_pin,newx-px>0?HIGH:LOW);
  digitalWrite(motors[1].dir_pin,newy-py>0?HIGH:LOW);
  if (MODE ==1) { // defines move in case of 2+2 axis or 4 independent axis
    digitalWrite(motors[2].dir_pin,newz-pz>0?HIGH:LOW);
    digitalWrite(motors[3].dir_pin,newe-pe>0?HIGH:LOW);
  } else {
    digitalWrite(motors[2].dir_pin,newx-px>0?HIGH:LOW);
    digitalWrite(motors[3].dir_pin,newy-py>0?HIGH:LOW);
  }


  float XYtimeMove;
  XYtimeMove = Time_Move (abs(newx-px),abs(newy-py),F);
  float ZEtimeMove;
  ZEtimeMove = Time_Move (abs(newz-pz),abs(newe-pe),F); 
  float OpTimeMove;
  OpTimeMove = max(XYtimeMove,ZEtimeMove);

 

  a[0].stepDelay = Step_Delay (a[0].nsteps, OpTimeMove);
  a[1].stepDelay = Step_Delay (a[1].nsteps, OpTimeMove);
  if (MODE ==1) {
    a[2].stepDelay = Step_Delay (a[2].nsteps, OpTimeMove);
    a[3].stepDelay = Step_Delay (a[3].nsteps, OpTimeMove);
  } else {
    a[2].stepDelay = Step_Delay (a[0].nsteps, OpTimeMove);
    a[3].stepDelay = Step_Delay (a[1].nsteps, OpTimeMove);
  }
  
  a[0].nextMove = 0;
  a[1].nextMove = 0;
  a[2].nextMove = 0;
  a[3].nextMove = 0;

  
  long i,j,iteration=0;
  long timeMove=0;
  long minTimeMove= 0;
  unsigned long Start_time, End_time, Elapsed_time, Actual_end_time =0;


  // for (j=0;j<NUM_AXIES;++j) {
  //   if(minTimeMove < a[j].stepDelay) {
  //     minTimeMove = a[j].stepDelay;
  //   }
  // }
  Serial.println("Step delay in milliseconds");
  Serial.println(a[0].stepDelay);


  Start_time = millis();
  End_time = Start_time + OpTimeMove*1000;
  StepsCount = 0;



  DebugCounter = 0;
  while (millis() <= End_time) {
    for(j=0;j<NUM_AXIES;++j) {
      if (a[j].stepDelay !=0) {              
        Elapsed_time = millis() - Start_time;
        if(Elapsed_time>a[j].nextMove) {
 
          DebugCounter++;
          // Serial.print("counter: ");
          // Serial.print(DebugCounter);
          // Serial.print(" Elapsed: ");
          // Serial.print(Elapsed_time);
          // Serial.print(" Nextmove: ");
          // Serial.print(a[j].nextMove);
          // Serial.print("\n");

          MotorStep(j);
          a[j].nextMove = a[j].nextMove + a[j].stepDelay;         
        }
      }
    }
  }
  Actual_end_time = millis();
  position(newx,newy,newz,newe);

  #ifdef VERBOSE
  Serial.println("move called");
  Serial.print(F("Number steps: "));
  Serial.println(a[0].nsteps);
  Serial.print(F("Operating time seconds: "));
  Serial.println(OpTimeMove);
  Serial.println(F("Steps per second: "));
  Serial.println(a[0].nsteps/OpTimeMove);
  Serial.println(F("Delay: "));
  Serial.println(a[0].stepDelay);
  Serial.println("Start time");
  Serial.println(Start_time);
  Serial.println("End_time");
  Serial.println(End_time);
  Serial.println("Actual End time");
  Serial.println(Actual_end_time);
  Serial.println(" Counter");
  Serial.println(StepsCount);
  #endif


}



 /**
  * Takes the axis couple and identifies the move time  
  */

float Time_Move (float D1, float D2, float UserFeed) {
  float MytimeMove;
  MytimeMove = sqrt(sq(D1) + sq(D2))/UserFeed;
  return MytimeMove;
}


/**
 * Calculates delay in microseconds to implement coordinated 2-axis move
 */
double Step_Delay (float steps, float timeMove) {


  double motorDelay;
  if (steps==0) {
    motorDelay = 0;
  } else {
    motorDelay = (timeMove/steps)*1000;
  }

 
  return motorDelay; 
}

/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}




/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  Serial.print("X");
  Serial.print(px);
  Serial.print(" ");
  Serial.print("Y");
  Serial.print(py);
  Serial.print(" ");
  Serial.print("Z");
  Serial.print(pz);
  Serial.print(" ");
  Serial.print("E");
  Serial.print(pe);
  Serial.print(" ");
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */

void homing() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    digitalWrite(motors[i].dir_pin,LOW);
    while (digitalRead(motors[i].limit_switch_pin))
    {
      MotorStep(i);
      delay(5);
    }
  }
 
}










