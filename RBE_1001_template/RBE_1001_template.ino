#include <PPM.h>
#include <Servo.h>

//chooses between red and blue auto
const int AUTO_PIN = A2;

//values used in teleop as the power of each motors
//initialized to stop
int left_drive = 90;
int right_drive = 90;
int flap_drive = 90;
int elevator_drive = 90;
int lifter_drive = 0; //intialized at down position

//lifter
const int LIFTER_PIN = 11;
Servo lifter;

//Elevator
Servo elevator;
const int ELEVATOR_PIN = 6;
const int ELEVATOR_SPEED = 120;

//bump sensors
const int LEFT_BUMP_PIN = 24;
const int RIGHT_BUMP_PIN = 25;

//rangefinder and distances used in autonomous
const int FRONT_RANGEFINDER_PIN = A0;
const int BACK_RANGEFINDER_PIN = A1;
const int WALL_DIST = 315;
const int SCORING_WALL_DIST = 390;
const int PIN_DIST = 500;
const int FAR_WALL_DIST = 175;
const int PIN_TURN_DIST = 370;
const int REV_DIST = 30;
const int TOLERANCE = 13;
const int TRACK_DIST = 195;
const float kPWall = 0.3;
const float SAMPLE_SIZE = 20;

//line sensors--not used in our autonomous programs
const int LEFT_SENSOR = 48;
const int RIGHT_SENSOR = 49;
boolean lastR,lastL;

//Drive train
int LEFT_MOTOR_PIN = 8;
int RIGHT_MOTOR_PIN = 7;
Servo left;
Servo right;

//limit switch to detect when we're lined up with the shabangabang
const int SCORING_LIMIT_PIN = 23;

//flap that flips out in autonomous to extend our reach
//this allows us to drop the bass and score the balls in the shabangabang within the time limit
const int FLAP_PIN = 9;
Servo flap;

//Controller information
PPM ppm(2);
const int LEFT_JOYSTICK = 3;
const int RIGHT_JOYSTICK = 2;
const int LEFT_BUTTON = 5;
const int RIGHT_BUTTON = 6;

//Autonomous is controlled by a statemachine
//Autonomous strategy is to drop the bass from the front edge
//  then collect balls as it drives to the shabangabang
//  then score those balls in the shabangabang
//States are re-used across our two autonomous programs
enum STATE {
  LIFTING_FLAP, DROPPING_FLAP, DRIVING, DROPPING, REVERSING, TURNING_TO_COLLECT, COLLECTING, TURNING_TO_SCORE, SCORING, DONE};
STATE state;

int t0=0;
int dt=0;

void setup() {
  Serial.begin(9600);
  elevator.attach(ELEVATOR_PIN);
  right.attach(RIGHT_MOTOR_PIN);
  left.attach(LEFT_MOTOR_PIN);
  flap.attach(FLAP_PIN);
  lifter.attach(LIFTER_PIN,1000,2000); //393 motors require these parameters
  flap.write(180);
  lifter.write(90);
  pinMode(LEFT_BUMP_PIN,INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN,INPUT_PULLUP);  
  pinMode(LEFT_SENSOR,INPUT_PULLUP);
  pinMode(RIGHT_SENSOR,INPUT_PULLUP);
  pinMode(SCORING_LIMIT_PIN,INPUT_PULLUP);  
}

void autonomous(unsigned long time){
  while( ppm.getChannel(1) == 0) {
  } // waits for controller to be turned on
  unsigned long startTime = millis();
  time = time * 1000;
  t0 = millis();

  //our robot can perform two autonomous routines
  //a potentiometer on the robot is used to decide between them
  //the lower half of the pot runs the red side auto, and the upper half runs the blue side auto
  boolean runRedAuto = analogRead(AUTO_PIN) > 512;

  //the two auto programs start at different states and are controlled by different state machines
  //this initializes the state machines to the right spot for each side
  if (runRedAuto){
    state = DRIVING;
  }
  else{
    state = LIFTING_FLAP;    
  }


  while ( millis() - startTime <= time){
    if (runRedAuto){
      redAuto();
    }
    else {
      blueAuto();
    }
  }
}

void teleop(unsigned long time){
  unsigned long startTime = millis();
  time = time * 1000;
  while ( millis() - startTime <= time){
    left_drive = ppm.getChannel(LEFT_JOYSTICK);
    right_drive = ppm.getChannel(RIGHT_JOYSTICK);

    left.write(180 - left_drive);  //drive the left motor corresponding to the controller
    right.write(right_drive); //drive the left motor corresponding to the controller
    //note that it is (180 - rightdrive)... this is because the motors must drive the same direction,
    //despite one motor being mounted backwards. 

    //remap feeder to slow it down
    elevator_drive = map(ppm.getChannel(LEFT_BUTTON),0,180,ELEVATOR_SPEED,180-ELEVATOR_SPEED);
    elevator.write(elevator_drive);

    //lifter is used to lift the shabangabang
    lifter_drive = ppm.getChannel(RIGHT_BUTTON);
    lifter.write(lifter_drive);

    //lifting the flap up keeps it folded in and out of the way    
    liftFlap();

  }
  exit(0);
}

void loop() {
  autonomous(20);
  teleop(150);
}

//////////////////////////////////////////
/////////////// AUTONOMOUS ///////////////
//////////////////////////////////////////

//this autonomous drives based on IR Rangefinder to the back wall
//turns based on IR rangefinder on the side to align to the pin
//drives for a set amout of time, backs up, turns until the bumper switch is hit
//the bumper switch tells us we're aligned with the wall
//the robot the drives forward and collects balls
void redAuto(){

  //watches for stop conditions and changes state appropriately
  updateRedState();

  switch(state){
  case DRIVING:
    setMotors(100,100);
    break;
  case TURNING_TO_SCORE:
    turnCounterClockwise;  
    break;
  case SCORING:
    driveToScore();
    break;
  case REVERSING:
    setMotors(-30,-30);
    liftFlap();
    break;
  case TURNING_TO_COLLECT:
    turnCounterClockwise();
    break;
  case COLLECTING:
    collect();
    break;
  case DONE:
    setMotors(0,0);
    break;
  }
}

//This autonomous runs on the blue side, and scores balls
//It releases the flap, drives a distance to the wall and lifts the flap
//this drops the bass and releases the balls
//It then drives back a distance, again based on IR, and turns until the bump sensors indicates we're flat to the wall
//It then drives, collecting balls and stopping a distance from the back wall
//it then turns to face the shabangabang based on IR distance from the side of the robots
//lastly, it drive into the shabangabang to score, stopping when the limit switch hits the shabangabang
void blueAuto() {

  //watches for stop conditions and changes state appropriately  
  updateBlueState();

  switch(state){
  case LIFTING_FLAP:
    halfLiftFlap();
    break;
  case DROPPING_FLAP:
    dropFlap();
    break;  
  case DRIVING:
    setMotors(50,50);    
    break;
  case DROPPING:
    liftFlap();
    setMotors(0,0);    
    break;
  case REVERSING:
    trackWall(-35);
    break;
  case TURNING_TO_COLLECT:
    turnCounterClockwise();
    break;
  case COLLECTING:
    collect();
    break;
  case TURNING_TO_SCORE:
    liftFlap();  
    turnCounterClockwise();
    break;
  case SCORING:
    elevator.write(90);
    driveToScore();
    break;
  case DONE:
    setMotors(0,0);
  }
}

///////NAVIGATION AND MANIPULATION///////////

//take power from -100 to 100, and follow wall at TRACK_DIST
//this uses Proportional control the follow a wall using the IR rangefinder on the side of our robot
void trackWall(int power){
  float distance_to_wall = range(BACK_RANGEFINDER_PIN);
  float error = distance_to_wall - TRACK_DIST;
  setMotors(power+kPWall*error,power-kPWall*error);
}

void driveToScore(){
  setMotors(60,60); //just enough to get over the bump on the shabangabang
}

void turnCounterClockwise(){
  setMotors(-50,50);
}

//drives forward and runs elevator to collect balls
void collect(){
  setMotors(80,80);
  elevator.write(ELEVATOR_SPEED);
}

void halfLiftFlap(){
  flap.write(90);
}

void liftFlap(){
  flap.write(0);
}


void dropFlap(){
  flap.write(180);
}

//enter left and right speed from -100 to 100
//this function removes the need for "magic numbers", because now all setMotors calls are readable as speed percentages
void setMotors(int l, int r){
  left.write(map(l,-100,100,0,180));
  right.write(map(r,-100,100,180,0));
}

///////STATE MACHINE CONTROL///////////

void updateRedState(){

  //dt is the time the robot has been in each state
  //allows states based on time
  dt = millis() - t0;

  //each state has a stop condition
  //at end of state timer resets and state is changed
  switch(state){
  case DRIVING:
    if (atFarWall()){
      t0=millis();
      state = TURNING_TO_SCORE;

    }
    break;
  case TURNING_TO_SCORE:
    if (alignedToPin()){
      t0=millis();
      state = SCORING;
    }
    break;
  case SCORING:
    if (dt > 3000){
      state=REVERSING;
      t0=millis();
    }
    break;    
  case REVERSING:
    if (dt>2000){
      t0=millis();      
      state = TURNING_TO_COLLECT;
    }
    break;
  case TURNING_TO_COLLECT:
    if (flatToWall()){
      t0=millis();      
      state = COLLECTING;  
    }
    break;
  case COLLECTING:
    if (dt>3000){
      t0=millis();
      state = DONE;
    }
    break;
  }
}

void updateBlueState(){
  //time for each state
  //use t0=millis() at the end of each state to reset the time

    //dt is the time the robot has been in each state
  //allows states based on time  
  dt = millis()- t0;

  //each state has a stop condition
  //at end of state timer resets and state is changed
  switch(state){
  case LIFTING_FLAP:
    if (dt>1000){
      state = DROPPING_FLAP;
      t0=millis();
    }
    break;
  case DROPPING_FLAP:
    if (dt>1000){
      state = DRIVING;
      t0=millis();
    }
    break;
  case DRIVING:
    if (atWall()){
      state = DROPPING;
      t0=millis();
    }
    break;
  case DROPPING:
    if (dt>1000){
      state = REVERSING;
      t0=millis();
    }
    break;
  case REVERSING:
    if (doneReversing() || dt > 2000){
      t0=millis();
      state=TURNING_TO_COLLECT;
    }
    break;
  case TURNING_TO_COLLECT:
    if (flatToWall()){
      t0=millis();
      state=COLLECTING;
    }  
    break;
  case COLLECTING:
    if (atFarWall()){
      t0=millis();
      state = TURNING_TO_SCORE;
    }
    break;
  case TURNING_TO_SCORE:
    if (alignedToWall()){
      t0=millis();
      state = SCORING;
    }
    break;
  case SCORING:
    if (atShabangabang()){
      state = DONE;
    }
    break;
  }
}

///////AUTONOMOUS CONTROL LOGIC///////////

//these functions range the IR sensor, and determine if the robot is at a certin distance
//some check the front rangefinder, some check the back (rear-side) rangefinder
//all are used as spot conditions for various autonomous states
boolean alignedToWall(){
  return abs(range(BACK_RANGEFINDER_PIN) - SCORING_WALL_DIST) < TOLERANCE;
}

boolean alignedToPin(){
  return abs(range(BACK_RANGEFINDER_PIN) - PIN_TURN_DIST) < TOLERANCE;
}

boolean doneReversing(){
  return abs(range(FRONT_RANGEFINDER_PIN) - REV_DIST) < TOLERANCE;  
}

boolean atFarWall(){
  return abs(range(FRONT_RANGEFINDER_PIN) - FAR_WALL_DIST) < TOLERANCE;
}

boolean atWall(){
  return abs(range(FRONT_RANGEFINDER_PIN) - WALL_DIST) < TOLERANCE;
}

boolean atShabangabang(){
  return digitalRead(SCORING_LIMIT_PIN) == LOW;
}

//takes a analog pin and returns the distance of a 50 size sample on that rangefinder
float range(const int PIN){
  int i,sum=0;
  float avg;
  for (i=0;i<SAMPLE_SIZE;i++){
    sum+=analogRead(PIN);
  }
  avg = sum/SAMPLE_SIZE;
  return avg;
}

boolean flatToWall(){
  return digitalRead(LEFT_BUMP_PIN) == LOW; //&& digitalRead(RIGHT_BUMP_PIN) == LOW;
}


