#include <Servo.h>

Servo left; // declare a servo to use for our left motor
Servo right;  //declare a servo to use for our right motor
Servo feeder;  //declare a servo to use for our right motor

//LINE FOLLOWING
int leftMotorPin = 6;
int rightMotorPin = 7;
int leftSensorPin = 26;
int rightSensorPin = 27;
boolean lastR,lastL;

//Autonomous is controlled by a statemachine
//Autonomous strategy is to line track and drop the bass
//  then collect balls down and back the course
//  then score those balls in the shabangabang
enum STATE {
  STARTING, TRACKING, REVERSING, FWD_COLLECTING, BACK_COLLECTING, TURNING, SCORING};
STATE state = STARTING;
int t0;

void setup(){
  left.attach(leftMotorPin);
  right.attach(rightMotorPin);
  pinMode(leftSensorPin, INPUT_PULLUP);
  pinMode(rightSensorPin, INPUT_PULLUP);
  t0 = millis();
}


void loop(){

  updateState();

  switch(state){
  case STARTING: 
    //drive forward to get to the line
    setMotors(100,100);
    break;
  case      TRACKING: 
    int sensorL = digitalRead(leftSensorPin);
    int sensorR = digitalRead(rightSensorPin);

    //uses sensor values to navigate line
    lineTrack(sensorR, sensorL);    
    break;
  case      REVERSING: 
    setMotors(-30,-100);
    break;
  case      FWD_COLLECTING: 
    setMotors(20,20);
    break;
  case      BACK_COLLECTING: 
    setMotors(-100,-100);  
    break;
  case      TURNING: 
    setMotors(100,30);
    break;
  case      SCORING: 
    setMotors(40,40);  
    break;
  }



}

void updateState(){
  int dt = millis()- t0;
  if (dt > 1000  && state == STARTING){
    state++; //move on to next state
    t0 = millis();
  }
  if (droppedBass() && state == TRACKING){
    state++;
    t0 = millis();    
  }
  if (dt > 3000 && state ==   REVERSING){
    state++;
    t0 = millis();        
  }
  if ( getDistance() < 20 && state ==   FWD_COLLECTING){
    state++;
    t0 = millis();        
  }
  if ( dt > 5000 && state ==   BACK_COLLECTING){
    state++;
    t0 = millis();        
  }
  if ( dt > 1000 && state ==   TURNING){
    state++;
    t0 = millis();        
  }
}

boolean droppedBase(){
  return false;
}

//get IR distance in CM
int getDistance(){
  return 50;
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





