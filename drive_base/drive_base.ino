#include <Servo.h>
#include <PPM.h>

PPM ppm(2); //this is used to attach and set up the Vex controller onto pin INT 21
// ewfwe

Servo left; // declare a servo to use for our left motor
Servo right;  //declare a servo to use for our right motor
Servo feeder;  //declare a servo to use for our right motor 
int leftdrive = 90;  //inititialize the motor drive constants to "stopped"
int rightdrive = 90;  //inititialize the motor drive constants to "stopped"

void setup(){

  Serial.begin(115200);  //this begins
  left.attach(6);		//this will attach the left motor to pin 5.
  right.attach(7);	//this will attach the right motor to pin 4.
  feeder.attach(8,1000,2000);  
}
void loop(){
  //this loop will occurr continuously - delivering the PWM commands from the controller to the motors.
  leftdrive = ppm.getChannel(3); //retreive the data from channel 3 on the controller
  rightdrive = ppm.getChannel(2); //retreive the data from channel 2 on the controller

  left.write(180 - leftdrive);  //drive the left motor corresponding to the controller
  right.write(rightdrive); //drive the left motor corresponding to the controller
  //note that it is (180 - rightdrive)... this is because the motors must drive the same direction,
  //despite one motor being mounted backwards. 

  int feeder_speed = ppm.getChannel(5);
  feeder.write(feeder_speed);

}

