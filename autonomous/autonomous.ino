#include <Servo.h>

//kill switch
const int KILL_PIN = 22;
boolean cancelled = false;

//Elevator
Servo elevator;
const int ELEVATOR_PIN = 6;


//rangefinder
const int FRONT_RANGEFINDER_PIN = A0;
const int BACK_RANGEFINDER_PIN = A1;
const int WALL_DIST = 300;
const int REV_DIST = 135;
const int TOLERANCE = 10;

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

//Autonomous is controlled by a statemachine
//Autonomous strategy is to drop the bass from the front edge
//  then collect balls down and back the course
//  then score those balls in the shabangabang
enum STATE {
  LIFTING_FLAP, DROPPING_FLAP, DRIVING, DROPPING, REVERSING, TURNING_TO_COLLECT, COLLECTING, TURNING_TO_SCORE, SCORING};
STATE state = LIFTING_FLAP;

int t0;

void setup(){
  Serial.begin(9600);
  elevator.attach(ELEVATOR_PIN);
  right.attach(RIGHT_MOTOR_PIN);
  left.attach(LEFT_MOTOR_PIN);
  flap.attach(FLAP_PIN);
  pinMode(LEFT_SENSOR,INPUT_PULLUP);
  pinMode(RIGHT_SENSOR,INPUT_PULLUP);
  pinMode(KILL_PIN,INPUT_PULLUP);
  pinMode(SCORING_LIMIT_PIN,INPUT_PULLUP);  
  t0 = millis();
  
  while (digitalRead(KILL_PIN));
  delay(2000);
}


void loop(){

  if (digitalRead(KILL_PIN) == LOW){
    cancelled = true;
  }

  if (!cancelled){
    fullRoutine();
  }
  else {
    flap.write(90);
    right.write(90);
    left.write(90);
    elevator.write(90);    
  }
}

void fullRoutine(){
  updateState();

  switch(state){
  case LIFTING_FLAP:
    liftFlap();
    break;
  case DROPPING_FLAP:
    dropFlap();
    break;  
  case DRIVING:
    setMotors(45,45);
    break;
  case DROPPING:
    liftFlap();
    setMotors(0,0);    
    break;
  case REVERSING:
    setMotors(-40,-40);
    break;
  case TURNING_TO_COLLECT:
    setMotors(-180,160);
    break;
  case COLLECTING:
    setMotors(85,55);
    elevator.write(125);
    break;
  case TURNING_TO_SCORE:
    elevator.write(90);  
    setMotors(-80,80);
    break;
  case SCORING:
    setMotors(0,0);
    break;    
  }
}

void updateState(){
  //time for each state
  //use t0=millis() at the end of each state to reset the time
  
  int dt = millis()- t0;
  
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
    if (doneReversing() && dt>1000 ){
      t0=millis();
      state=TURNING_TO_COLLECT;
    }
    break;
  case TURNING_TO_COLLECT:
    if (dt>3000){
      t0=millis();
      state=COLLECTING;
    }  
    break;
  case COLLECTING:
    if (atWall() && dt>5000){
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
    break;    
  }
}

//turns until the back sensor reads a certain distance
boolean alignedToWall(){
  return abs(range(BACK_RANGEFINDER_PIN) - WALL_DIST) < TOLERANCE;
}

void liftFlap(){
  flap.write(10);
}


void dropFlap(){
  flap.write(180);
}

boolean droppedBass(){
  return false;
}

boolean doneReversing(){
  return abs(range(FRONT_RANGEFINDER_PIN) - REV_DIST) < TOLERANCE;  
}


boolean atWall(){
  return abs(range(FRONT_RANGEFINDER_PIN) - WALL_DIST) < TOLERANCE;
}

//overall strategy is:
//if you see double black, go full
//if you see white on left, turn right until you see white on right
//if you see white on right, turn left until you see white on left
void lineTrack(int R, int L){
  if (R && L){
    forward();
  }
  else if (!L&&lastR){
    turnRight();
  }
  else if (!R&&lastL){
    turnLeft();
  }


  lastR=R;
  lastL=L;

  delay(20);
}

//takes a analog pin and returns the distance of a 50 size sample on that rangefinder
float range(const int PIN){
  int i,sum=0;
  float avg;
  for (i=0;i<50;i++){
    sum+=analogRead(PIN);
  }
  avg = sum/50.0;
  return avg;
}

//equal speed to motors
void forward(){
  setMotors(100,100);
}

//turns heavy on left wheel, light on right wheel
void turnRight(){
  setMotors(100,15);
}

//turns heavy on right wheel, light on left wheel
void turnLeft(){
  setMotors(14,100);
}

//enter left and right speed from -100 to 100
void setMotors(int l, int r){
  left.write(map(l,-100,100,0,180));
  right.write(map(r,-100,100,160,30)); //compensate because other motor is weaker
}
