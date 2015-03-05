#include <PPM.h>
#include <Servo.h>

//inititialize the motor drive constants to "stopped"
int left_drive = 90;
int right_drive = 90;
int flap_drive = 90;
int elevator_drive = 70;

//kill switch
const int KILL_PIN = 22;
boolean cancelled = false;

//Elevator
Servo elevator;
const int ELEVATOR_PIN = 6;

//bump sensors
const int LEFT_BUMP_PIN = 24;
const int RIGHT_BUMP_PIN = 25;

//rangefinder
const int FRONT_RANGEFINDER_PIN = A0;
const int BACK_RANGEFINDER_PIN = A1;
const int WALL_DIST = 315;
const int SCORING_WALL_DIST = 390;
const int FAR_WALL_DIST = 175;
const int REV_DIST = 30;
const int TOLERANCE = 13;
const int TRACK_DIST = 195;
const float kPWall = 0.3;
const float SAMPLE_SIZE = 20;

//line sensors
//1 or HIGH means white
const int LEFT_SENSOR = 48;
const int RIGHT_SENSOR = 49;
boolean lastR,lastL;

//Drive train
int LEFT_MOTOR_PIN = 8;
int RIGHT_MOTOR_PIN = 7;
Servo left;
Servo right;

//limit switch
const int SCORING_LIMIT_PIN = 23;

//flap
const int FLAP_PIN = 9;
Servo flap;

//Controller
PPM ppm(2);
const int LEFT_JOYSTICK = 3;
const int RIGHT_JOYSTICK = 2;
const int LEFT_BUTTON = 5;
const int RIGHT_BUTTON = 6;

//Autonomous is controlled by a statemachine
//Autonomous strategy is to drop the bass from the front edge
//  then collect balls down and back the course
//  then score those balls in the shabangabang
enum STATE {
  LIFTING_FLAP, DROPPING_FLAP, DRIVING, DROPPING, REVERSING, TURNING_TO_COLLECT, COLLECTING, TURNING_TO_SCORE, SCORING, DONE};
STATE state = LIFTING_FLAP;

int t0=0;
int dt=0;

void setup() {
  elevator.attach(ELEVATOR_PIN);
  right.attach(RIGHT_MOTOR_PIN);
  left.attach(LEFT_MOTOR_PIN);
  flap.attach(FLAP_PIN);
  flap.write(180);
  pinMode(LEFT_BUMP_PIN,INPUT_PULLUP);
  pinMode(RIGHT_BUMP_PIN,INPUT_PULLUP);  
  pinMode(LEFT_SENSOR,INPUT_PULLUP);
  pinMode(RIGHT_SENSOR,INPUT_PULLUP);
  pinMode(KILL_PIN,INPUT_PULLUP);
  pinMode(SCORING_LIMIT_PIN,INPUT_PULLUP);  
}

void autonomous(unsigned long time){
  while( ppm.getChannel(1) == 0) {
  } // waits for controller to be turned on
  unsigned long startTime = millis();
  time = time * 1000;
  t0 = millis();
  while ( millis() - startTime <= time){
    autoRoutine();
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
    elevator_drive = map(ppm.getChannel(LEFT_BUTTON),0,180,50,130);
    elevator.write(elevator_drive);
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


void autoRoutine() {
  updateState();

  switch(state){
  case LIFTING_FLAP:
    halfLiftFlap();
    break;
  case DROPPING_FLAP:
    dropFlap();
    break;  
  case DRIVING:
    setMotors(50,58);
    break;
  case DROPPING:
    liftFlap();
    setMotors(0,0);    
    break;
  case REVERSING:
    trackWall(-35);
    break;
  case TURNING_TO_COLLECT:
    setMotors(-50,50);
    break;
  case COLLECTING:
    setMotors(65,73+dt/450); //this makes it curve more as it goes
    elevator.write(125);
    break;
  case TURNING_TO_SCORE:
    liftFlap();  
    setMotors(-36,40);
    break;
  case SCORING:
    elevator.write(90);
    setMotors(60,60);
    break;
  case DONE:
    left.write(90);
    right.write(90);
  }
}


//take power from -100 to 100, and follow wall at TRACK_DIST
void trackWall(int power){
  float distance_to_wall = range(BACK_RANGEFINDER_PIN);
  float error = distance_to_wall - TRACK_DIST;
  Serial.println(error*kPWall);
  setMotors(power+kPWall*error,power-kPWall*error);
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
void setMotors(int l, int r){
  left.write(map(l,-100,100,0,180));
  right.write(map(r,-100,100,160,30)); //compensate because other motor is weaker
}

void updateState(){
  //time for each state
  //use t0=millis() at the end of each state to reset the time

  dt = millis()- t0;

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
      pulseStopFwd();
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
      pulseStopRev();
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

//pulses reverse for a sec to stop it
void pulseStopFwd(){
  setMotors(30,30);
  delay(50);
}      

void pulseStopRev(){
  setMotors(-30,-30);
  delay(50);
}      

//turns until the back sensor reads a certain distance
boolean alignedToWall(){
  return abs(range(BACK_RANGEFINDER_PIN) - SCORING_WALL_DIST) < TOLERANCE;
}

boolean atShabangabang(){
  return digitalRead(SCORING_LIMIT_PIN) == LOW;
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

boolean droppedBass(){
  return false;
}

