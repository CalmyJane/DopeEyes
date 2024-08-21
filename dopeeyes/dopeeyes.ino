#include <Servo.h>


Servo servo_upperlid;  // create servo object to control a servo
Servo servo_lowerlid;  // create servo object to control a servo
Servo servo_updown;  // create servo object to control a servo
Servo servo_leftright;  // create servo object to control a servo

// twelve servo objects can be created on most boards


int pos = 0;    // variable to store the servo position


void setup() {

  servo_upperlid.attach(2);  // attaches the servo on pin 9 to the servo object
  servo_lowerlid.attach(3);  // attaches the servo on pin 9 to the servo object
  servo_updown.attach(4);  // attaches the servo on pin 9 to the servo object
  servo_leftright.attach(5);  // attaches the servo on pin 9 to the servo object

}


void loop() {

  for (pos = 60; pos <= 120; pos += 1) { // goes from 0 degrees to 180 degrees

    // in steps of 1 degree

    servo_upperlid.write(pos);              // tell servo to go to position in variable 'pos'
    servo_lowerlid.write(pos);              // tell servo to go to position in variable 'pos'
    servo_updown.write(pos);              // tell servo to go to position in variable 'pos'
    servo_leftright.write(pos);              // tell servo to go to position in variable 'pos'

    delay(15);                       // waits 15ms for the servo to reach the position

  }

  for (pos = 120; pos >= 60; pos -= 1) { // goes from 180 degrees to 0 degrees

    servo_upperlid.write(pos);              // tell servo to go to position in variable 'pos'
    servo_lowerlid.write(pos);              // tell servo to go to position in variable 'pos'
    servo_updown.write(pos);              // tell servo to go to position in variable 'pos'
    servo_leftright.write(pos);              // tell servo to go to position in variable 'pos'

    delay(15);                       // waits 15ms for the servo to reach the position

  }

}