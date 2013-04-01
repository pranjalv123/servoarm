// code derived from http://www.instructables.com/id/Robotic-Arm-with-Servo-Motors/?ALLSTEPS
// code derived from http://www.instructables.com/id/Intro-and-what-youll-need/?ALLSTEPS
// derived from http://arduino.cc/en/Tutorial/Graph
//l1 67.5 mm
//l2 65.8mm 

#include <Servo.h>
Servo servo1;
Servo servo2;

double theta1_neg = 0;      // joint limits of robotic arm using right hand rule for sign
double theta1_pos = 130;  // in servo degrees (between 0 to 180 for the sg90 servo)
double theta2_neg = 0;
double theta2_pos = 150;

double highY = 80; // line drawing targets in mm
double lowY = 20;
double staticX = 20;

boolean elbowup = true; // true=elbow up, false=elbow down

double computedX = 0.0; //results of forward kinematics
double computedY = 0.0;
double c2 = 0.0; // is btwn -1 and 1
double s2 = 0.0;

double theta1 = 0.0;  // target angles as determined through IK
double theta2 = 0.0;
double l1 = 67.5; //length of links in mm
double l2 = 65.8;
double y = 0.0;

double map1 = 0.0; //if need to scale IK output to servo.write() values
double map2 = 0.0;

void setup()
{
    Serial.begin(9600);
    servo1.attach(2,500,2400);
    servo2.attach(3,500,2400);
}

void input()    // this is the subroutine that waits for the user to hit enter //not used
  {
  int incomingByte = 0;	// for incoming serial data
  Serial.println("Hit return for next motion");   
  while (incomingByte != 13)  // 13 is the ASCII code for "enter"
    {
	if (Serial.available() > 0) 
         {   
		// read the incoming byte:
		incomingByte = Serial.read();

		// say what you got:
		Serial.print("I received: ");
		Serial.println(incomingByte, DEC);
                //-servo1.write(incomingByte);
	 }
     }
  }
  
void loop()
{
//  move_to_start();
  line_y();
//  Serial.print("theta1 "); Serial.print(theta1);
//  Serial.print(" map1 "); Serial.print(map1 );
//  Serial.print(" theta2 "); Serial.print(theta2);
//  Serial.print(" map2 "); Serial.print(map2);
//  Serial.println();
}
void move_to_start() {
    get_angles(0, 0);
    drive_joints();
    delay(500);
//    get_angles(0,50);
//    drive_joints();
//    delay(500);
}

// Given theta1, theta2 solve for target(Px, Py) (forward kinematics)
void get_xy() {
  computedX = l1*cos(radians(theta1)) + l2*cos(radians(theta1+theta2));
  computedY = l1*sin(radians(theta1)) + l2*sin(radians(theta1+theta2));
}

// Given target(Px, Py) solve for theta1, theta2 (inverse kinematics)
void get_angles(double Px, double Py) {
  // first find theta2 where c2 = cos(theta2) and s2 = sin(theta2)
  c2 = (pow(Px,2) + pow(Py,2) - pow(l1,2) - pow(l2,2))/(2*l1*l2); // is btwn -1 and 1
  //Serial.print("c2 "); Serial.print(c2);

  if (elbowup == false) {
    s2 = sqrt(1 - pow(c2,2));  // sqrt can be + or -, and each corresponds to a different orientation
  }
  else if (elbowup == true) {
    s2 = -sqrt(1 - pow(c2,2));
  }
  theta2 = degrees(atan2(s2,c2));  // solves for the angle in degrees and places in correct quadrant
  //theta2 = map(theta2, 0,180,180,0); // the servo is flipped. 0 deg is on the left physically
  // now find theta1 where c1 = cos(theta1) and s1 = sin(theta1)
  theta1 = degrees(atan2(-l2*s2*Px + (l1 +  l2*c2)*Py, (l1 + l2*c2)*Px + l2*s2*Py));
  theta1 = theta1;
  //Serial.println();
}

void drive_joints() { //just servo.write() in my case
  //constrained1 = constrain(theta1,theta1_pos,theta1_neg);
  //constrained2 = constrain(theta2,theta2_pos,theta2_neg);
  
  //map1 = map(theta1, -180,180, 0,130);
  //map2 = map(theta2, -180,180, 3,150);

//  servo1.write(map1);
//  servo2.write(map2);
//  
  servo1.write(theta1);
  servo2.write(theta2);
  
  //Serial.print("theta1,"); 
  Serial.print(theta1);
//  Serial.print("theta1 "); Serial.print(constrain(theta1,theta1_pos,theta1_neg));
  //Serial.print(",theta2,");
    Serial.print(" ");
  Serial.print(theta2);
  Serial.println();
  delay(10);
}



void line_y() {
  for(y = lowY; y < highY; y += 1) {  // draw straight line up
    get_angles(staticX,y);
//  Serial.print("x "); Serial.print(staticX);
//  Serial.print(" y "); Serial.print(y);
//  Serial.println();
    drive_joints();
  }
  for(y = highY; y > lowY; y -= 1) {  // draw straight line down
    get_angles(staticX,y);
//  Serial.print("x "); Serial.print(staticX);
//  Serial.print(" y "); Serial.print(y);
//  Serial.println(); 
    drive_joints();
  }
}
