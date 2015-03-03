#include <Servo.h>
#include <PPM.h>

//Controller
PPM ppm(2);
const int LEFT_JOYSTICK = 3;
const int RIGHT_JOYSTICK = 2;
const int LEFT_BUTTON = 5;
const int RIGHT_BUTTON = 6;

//Motors
Servo left;
Servo right;
Servo feeder;
Servo flap;

//inititialize the motor drive constants to "stopped"
int left_drive = 90;
int right_drive = 90;
int flap_drive = 90;
int elevator_drive = 90;

void setup(){
  Serial.begin(115200);
  feeder.attach(6);
  right.attach(7);
  left.attach(8);
  flap.attach(9);
  pinMode(LEFT_SENSOR,INPUT_PULLUP);
  pinMode(RIGHT_SENSOR,INPUT_PULLUP);
}

void loop(){
  left_drive = ppm.getChannel(LEFT_JOYSTICK);
  right_drive = ppm.getChannel(RIGHT_JOYSTICK);

  left.write(180 - left_drive);  //drive the left motor corresponding to the controller
  right.write(right_drive); //drive the left motor corresponding to the controller
  //note that it is (180 - rightdrive)... this is because the motors must drive the same direction,
  //despite one motor being mounted backwards. 

  //remap feeder to slow it down
  elevator_drive = map(ppm.getChannel(LEFT_BUTTON),0,180,50,130);
  feeder.write(elevator_drive);

  flap.write(0);

  Serial.print(digitalRead(LEFT_SENSOR));
  Serial.print("  ");
  Serial.println(digitalRead(RIGHT_SENSOR));  

}

