#include <Servo.h>

Servo left; // declare a servo to use for our left motor
Servo right;  //declare a servo to use for our right motor
Servo feeder;  //declare a servo to use for our right motor 

void setup(){

}


void loop(){
  
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
