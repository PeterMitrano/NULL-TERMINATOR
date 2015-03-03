/* This is the RBE 1001 Template as of 2/26/2015. This Template 
 is designed to run the autonomous and teleop sections of the final 
 competition. Write and test your autonomous and teleop code on your
 own, then simply copy and paste your code into the appropriate section 
 and make sure the time passed to each function corresponds with the
 time in seconds that each set of code should run. For example,
 autonomous(20); will run for 20 seconds after the transmitter is 
 turned on. The code will not start until the controller is turned on.
 There are print statements commented out that were used to test */
#include <Servo.h>
#include <PPM.h> // includes the PPM library


PPM ppm(2); // initializes a PPM object named ppm onto pin #2


Servo flap;

void setup() {
  Serial.begin(9600); // Starts the serial port
  flap.attach(9);
  flap.write(180);
  // Put your setup code here, to run once:

}




void autonomous(unsigned long time){ // function definition
  while( ppm.getChannel(1) == 0) {
  } // waits for controller to be turned on
  unsigned long startTime = millis(); // sets start time of autonomous
  time = time * 1000;  // modifies milliseconds to seconds
  while ( millis() - startTime <= time){// compares start time to time entered in the autonomous function

    // Enter Autonoumous User Code Here

      Serial.println("Autonoumous"); //prints Autonomous over serial (usb com port)
    delay(50); //delay to prevent spamming the serial port 

  }


}

void teleop(unsigned long time){ // function definition
  unsigned long startTime = millis(); // sets start time of teleop
  time = time * 1000; // Convert milliseconds to seconds
  while ( millis() - startTime <= time){ // Compare start time to time entered in the teleop function
  

    
    Serial.println("TeleOp"); //prints Teleop over serial (usb com port)
    delay(50); //delay to prevent spamming the serial port 

  }
  exit(0); // exits program
}


void loop() {

  autonomous(20); // Time in seconds to run autonomous code

  teleop(150);   // Time in seconds to run teleop code. 
  // We have 30 seconds extra in case the offical awards more time

}



