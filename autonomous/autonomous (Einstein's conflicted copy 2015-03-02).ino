#include <Servo.h>

//Elevator
Servo elevator;
const int ELEVATOR_PIN = 6;


//rangefinder
const int RANGEFINDER_PIN = A0;
const int WALL_DIST = 277;

//line sensors
//1 or HIGH means white
const int LEFT_SENSOR = 48;
const int RIGHT_SENSOR = 49;
boolean lastR,lastL;

//Drive train
int LEFT_MOTOR_PIN = 7;
int RIGHT_MOTOR_PIN = 8;
Servo left;
Servo right;

//flap
const int FLAP_PIN = 9;
Servo flap;

//Autonomous is controlled by a statemachine
//Autonomous strategy is to line track and drop the bass
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
  t0 = millis();
}


void loop(){

  updateState();
  
  Serial.println(state);

  switch(state){
  case LIFTING_FLAP:
    liftFlap();
    break;
  case DROPPING_FLAP:
    dropFlap();
    break;
  case DRIVING:
    setMotors(50,50);
    break;
  case DROPPING:
    liftFlap();
    break;
  case REVERSING:
    setMotors(-50,-50);
    break;
  case TURNING_TO_COLLECT:
    setMotors(-80,80);
    break;
  case COLLECTING:
    setMotors(100,100);
    elevator.write(130);
    break;
  case TURNING_TO_SCORE:
    setMotors(-80,80);
    break;
  case SCORING:
    setMotors(50,50);
    break;    
  }
}

void updateState(){
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
      //state = DRIVING;
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
    break;
  case TURNING_TO_COLLECT:
    break;
  case COLLECTING:
    break;
  case TURNING_TO_SCORE:
    break;
  case SCORING:
    break;    
  }
}

void liftFlap(){
  flap.write(0);
}


void dropFlap(){
  flap.write(180);
}

boolean droppedBass(){
  return false;
}

boolean atWall(){
  return analogRead(RANGEFINDER_PIN) < WALL_DIST;
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
  right.write(map(r,-100,100,160,20)); //compensate because other motor is weaker
}










