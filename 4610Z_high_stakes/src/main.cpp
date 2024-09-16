/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jameshou                                                  */
/*    Created:      8/23/2024, 9:46:39 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <iomanip>


using namespace vex;


// A global instance of competition
competition Competition;


brain Brain;


controller Controller1=controller(primary);
motor LeftFront=motor(PORT17, ratio6_1, true);
motor RightFront=motor(PORT13, ratio6_1, false);
motor LeftMiddle=motor(PORT16, ratio6_1, true);
motor RightMiddle=motor(PORT15, ratio6_1, false);
motor LeftRear=motor(PORT19, ratio6_1, false);
motor RightRear=motor(PORT20, ratio6_1, true);
motor Hook=motor(PORT14, ratio6_1, false);
motor Intake=motor(PORT2, ratio6_1, true);
motor Arm=motor(PORT5, ratio18_1, false);

digital_out Clamp=digital_out(Brain.ThreeWirePort.A);
// digital_out Claw = digital_out(Brain.ThreeWirePort.D);
// digital_out ClawFold = digital_out(Brain.ThreeWirePort.B);


rotation VertEncoder = rotation(PORT12, false);
inertial Gyro(PORT12);


// define your global instances of motors and other devices here


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

int autonNum = 1;
void drawScreen(){
 Brain.Screen.clearScreen();
 Brain.Screen.setFillColor(black);


 // ALL VALUES CHANGE BASED ON NUMBER OF AUTONS, BASING THIS OFF OF 2 AUTONS PER POSITION AND SKIPPING SKILLS WHICH WILL GO UNDER WHEN I GET TO IT
 int x1 = 5;
 int y1 = 50;
 int x2 = 50;
 int y2 = 70;

 int spacing = 60;
 int autonTotal = 8;
 color fill = red;
 color pen = red;
 color textHighlight= black;

 for (int i = 0; i < autonTotal; i++) {
   if (i+1 == autonNum){
     fill = green; 

     if (i+1 <= 3) {
       pen = red;
     }
     else if (i+1 > 3) {
       pen = blue;
     }
     else {
      pen = white;
     }
   }
   else {
     if (i+1 <= 2) {
       fill = red;
       pen = red;
     }
     else if (i+1 > 2) {
       fill = blue;
       pen = blue;
     }
     else {
      fill = black;
      pen = black;
     }
   }

   Brain.Screen.setPenColor(pen);
   Brain.Screen.drawRectangle(x1+(spacing*i), y1, x2, y2, fill);
 }


 while(true){
   int x = Brain.Screen.xPosition();
   int y = Brain.Screen.yPosition();


   if (y1 < y && y < y1+y2) {
     for (int i = 0; i < autonTotal; i++) {
       if (x1 + (spacing*i) < x && x < (spacing*(i+1))-x1) {
         if (autonNum >= 7){
           fill = black;
         }
         else if (autonNum <= 3) {
           fill = red;
         }
         else if (autonNum > 3) {
           fill = blue;
         }
         
         Brain.Screen.drawRectangle(x1+(spacing*(autonNum-1)),y1,x2,y2, fill);
        
         autonNum = i+1;
         fill = green;
         Brain.Screen.drawRectangle(x1+(spacing*(autonNum-1)), y1, x2, y2, fill);
       }
     }
   }

   Brain.Screen.setPenColor(white);
   Brain.Screen.setFillColor(black);

   Brain.Screen.printAt(20,90,"LQ");
   Brain.Screen.printAt(80,90,"RQ");
   Brain.Screen.printAt(140,90,"RR");
   Brain.Screen.printAt(200,90,"LQ");
   Brain.Screen.printAt(260,90, "RQ");
   Brain.Screen.printAt(320,90, "RR");
   Brain.Screen.printAt(380,90, "Pre");
   Brain.Screen.printAt(440,90, "N/A");

   Brain.Screen.setFillColor(black);
   Brain.Screen.setPenColor(green);

   Brain.Screen.printAt(5,30, "4610Z: Zenith");
   Brain.Screen.printAt(250,30, "Auton Number: %d", autonNum);

   Controller1.Screen.clearLine(2);
   Controller1.Screen.setCursor(2,1);

   if (autonNum == 1) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Left red side (front channel on tape)                          ");
     Controller1.Screen.print("Auton: Left Red Side (6pt)");
   }
   else if (autonNum == 2) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Right red side (front channel on tape)                 ");
     Controller1.Screen.print("Auton: Right Red Side (4pt)");
   }
   else if (autonNum == 3) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Right red rush (plastic ramp on tape)              ");
     Controller1.Screen.print("Auton: Right Rush (4pt)");
   }
   else if (autonNum == 4) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Left blue side (front channel on tape)             ");
     Controller1.Screen.print("Auton: Left Blue Side (4pt)");
   }
   else if (autonNum == 5) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Right blue side (front channel on tape)            ");
     Controller1.Screen.print("Auton: Right Blue Side (6pt)");
   }
   else if (autonNum == 6) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Left blue rush (plastic ramp on tape)                 ");
     Controller1.Screen.print("Auton: Left Blue Rush (4pt)");
   }
   else if (autonNum == 7) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Score preload                               ");
   }
   else if (autonNum == 8) {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description: Off the line                               ");
   }
   else {
     Brain.Screen.printAt(5,((y2-y1)+90+30), "Auton Description:                                             ");
   }
   wait(30, msec);
 }
}

void pre_auton(void) {
 Gyro.calibrate();
  while (Gyro.isCalibrating()) {
   wait(10,msec);
 }

 RightFront.setStopping(coast);
 RightRear.setStopping(coast);
 RightMiddle.setStopping(coast);
 LeftFront.setStopping(coast);
 LeftRear.setStopping(coast);
 LeftMiddle.setStopping(coast);
 RightFront.resetPosition();
 RightRear.resetPosition();
 RightMiddle.resetPosition();
 LeftFront.resetPosition();
 LeftRear.resetPosition();
 LeftMiddle.resetPosition();
 Intake.setVelocity(100, percent);
 Hook.setVelocity(100,percent);
 Arm.setVelocity(100,percent);
 Clamp.set(false);
 Hook.setStopping(coast);
 VertEncoder.resetPosition();
  drawScreen();


 // All activities that occur before the competition starts
 // Example: clearing encoders, setting servo positions, ...
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

double x = 0.0, y = 0.0, theta = 0.0, prevTheta = 0.0, RobotPosition;


// Constants
const double WHEEL_DIAMETER = 3.25; // Example wheel diameter in inches
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TICKS_PER_REV = 450.0; // Encoder ticks per wheel revolution

// Mutex for thread safety
vex::mutex positionMutex;

// Struct to hold odometry data
struct OdometryData {
 int x;
 int y;
 int heading;
 double RobotPosition;
};

// Function to update odometry
void updateOdometry() {
 while (true) {
   // Get the positions of the encoders
   //double EncPosition = VertEncoder.position(degrees) ;
   double EncPosition = (RightFront.position(degrees)+RightRear.position(degrees)+RightMiddle.position(degrees)+LeftFront.position(degrees)+LeftMiddle.position(degrees)+LeftRear.position(degrees))/6 ;
  


   // Calculate the distances each wheel has traveled in inches
  
  
   // Get the current orientation from the inertial sensor
   double currentTheta = Gyro.rotation(degrees) * M_PI / 180.0; // Convert to radians


   // Calculate the change in orientation
   double deltaTheta = currentTheta - prevTheta;


   RobotPosition = (EncPosition / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
  
   // Update x, y, and theta
   positionMutex.lock();
   theta = currentTheta;
   x = (RobotPosition * cos(theta));
   y = (RobotPosition * sin(theta));
   RobotPosition = ((EncPosition / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE);
   prevTheta = currentTheta;
   positionMutex.unlock();


   // Add a small delay to prevent CPU hogging
   this_thread::sleep_for(10);
 }
}

// Function to get the current position
OdometryData getOdometry() {
 OdometryData data;
 positionMutex.lock();
 data.x = (x);
 data.y =(y);
 data.RobotPosition=(RobotPosition);
 data.heading =(theta * 180.0 / M_PI); // Convert theta to degrees
 positionMutex.unlock();
 return data;
}

void drivePDC(double target, int angle) {
   double goal = (target + RobotPosition);
   double error = goal - RobotPosition;
   double ierror = goal - RobotPosition;
   double tError = angle - Gyro.rotation(degrees);


   double derivative = 0;
   double kP = 0.15;
   double kD = 0.1;
   int ktp = .75;
   double previousError = 0;
   double speedfactor;


   while (fabs(error) > .25) {
       // Update current position


       // Calculate errors
       error = goal - RobotPosition;
       tError = angle - Gyro.rotation(degrees);


       speedfactor= fabs(error)/(3.5*fabs(ierror));
        if (error>45){
            speedfactor=.25;
        }

       // Calculate derivative before using it
      
       derivative = (error*20) - previousError;
       double ioutput = ((error*20) * kP + derivative * kD);
       double output = ioutput*speedfactor;


       // Speed clamping
       if (output > 100) {
           output = 100;
       } else if (output < -100) {
           output = -100;
       } else if (output > 0 && output < 15) {
           output = 15;
       } else if (output < 0 && output > -15) {
           output = -15;
       }


       // Update previous error
       previousError = error;


       // Motor control with angle correction
       LeftRear.spin(forward, output+tError*ktp, pct);
       RightRear.spin(forward, output-tError*ktp, pct);
       LeftMiddle.spin(forward, output+tError*ktp, pct);
       RightMiddle.spin(forward, output-tError*ktp, pct);
       LeftFront.spin(forward, output+tError*ktp, pct);
       RightFront.spin(forward, output-tError*ktp, pct);
       wait(10, msec);
   }


   // Stop motors
   LeftRear.stop(brake);
   RightRear.stop(brake);
   LeftMiddle.stop(brake);
   RightMiddle.stop(brake);
   LeftFront.stop(brake);
   RightFront.stop(brake);
}

void drivePDCLong(double target, int angle) {
   double goal = (target + RobotPosition);
   double error = goal - RobotPosition;
   double ierror = goal - RobotPosition;
   double tError = angle - Gyro.rotation(degrees);


   double derivative = 0;
   double kP = 0.15;
   double kD = 0.1;
   int ktp = .75;
   double previousError = 0;
   double speedfactor;


   while (fabs(error) > .25) {
       // Update current position


       // Calculate errors
       error = goal - RobotPosition;
       tError = angle - Gyro.rotation(degrees);


       speedfactor= fabs(error)/(2.5*fabs(ierror));
        if (error>45){
            speedfactor=.25;
        }

       // Calculate derivative before using it
      
       derivative = (error*20) - previousError;
       double ioutput = ((error*20) * kP + derivative * kD);
       double output = ioutput*speedfactor;


       // Speed clamping
       if (output > 100) {
           output = 100;
       } else if (output < -100) {
           output = -100;
       } else if (output > 0 && output < 15) {
           output = 15;
       } else if (output < 0 && output > -15) {
           output = -15;
       }


       // Update previous error
       previousError = error;


       // Motor control with angle correction
       LeftRear.spin(forward, output+tError*ktp, pct);
       RightRear.spin(forward, output-tError*ktp, pct);
       LeftMiddle.spin(forward, output+tError*ktp, pct);
       RightMiddle.spin(forward, output-tError*ktp, pct);
       LeftFront.spin(forward, output+tError*ktp, pct);
       RightFront.spin(forward, output-tError*ktp, pct);
       wait(10, msec);
   }


   // Stop motors
   LeftRear.stop(brake);
   RightRear.stop(brake);
   LeftMiddle.stop(brake);
   RightMiddle.stop(brake);
   LeftFront.stop(brake);
   RightFront.stop(brake);
}

void stupidRun(double target) {
   double goal = (target + RobotPosition);
   double error = goal - RobotPosition;
   double ierror = goal - RobotPosition;


   double derivative = 0;
   double kP = 0.5;
   double kD = 0.25;
   double previousError = 0;


   while (fabs(error) > .5) {
       // Update current position


       // Calculate errors
       error = goal - RobotPosition;




       // Calculate derivative before using it
      
       derivative = (error*50) - previousError;
       double output = ((error*50) * kP + derivative * kD);


       // Speed clamping
       if (output > 100) {
           output = 100;
       } else if (output < -100) {
           output = -100;
       } else if (output > 0 && output < 10) {
           output = 10;
       } else if (output < 0 && output > -10) {
           output = -10;
       }


       // Update previous error
       previousError = error;


       // Motor control with angle correction
       LeftRear.spin(forward, output, pct);
       RightRear.spin(forward, output, pct);
       LeftMiddle.spin(forward, output, pct);
       RightMiddle.spin(forward, output, pct);
       LeftFront.spin(forward, output, pct);
       RightFront.spin(forward, output, pct);
       wait(10, msec);
   }


   // Stop motors
   LeftRear.stop(brake);
   RightRear.stop(brake);
   LeftMiddle.stop(brake);
   RightMiddle.stop(brake);
   LeftFront.stop(brake);
   RightFront.stop(brake);
}

void turnPD(int angle) {
   double derivative = 0;
   double kD = 0.25;
   double kP = 0.35;
   double previousError;


   double error = angle - Gyro.rotation(degrees);
    while (fabs(error) > 2.5) {
       error = angle - Gyro.rotation(degrees);


       int output = (error * kP + derivative * kD) ;


       // Speed clamping
       if (output > 100) {
           output = 100;
       } else if (output < -100) {
           output = -100;
       } else if (output > 0 && output <10) {
           output = 10;
       } else if (output < 0 && output > -10) {
           output = -10;
       }


       derivative = error - previousError;
       previousError = error;


       // Motor control
       LeftRear.spin(forward, output, pct);
       RightRear.spin(reverse, output, pct);
       LeftMiddle.spin(forward, output, pct);
       RightMiddle.spin(reverse, output, pct);
       LeftFront.spin(forward, output, pct);
       RightFront.spin(reverse, output, pct);


       wait(10, msec);
   }


   // Stop motors
   LeftRear.stop(brake);
   RightRear.stop(brake);
   LeftMiddle.stop(brake);
   RightMiddle.stop(brake);
   LeftFront.stop(brake);
   RightFront.stop(brake);
}

int brainDisplay(){
   while(true){
     // Get the current position
     OdometryData currentPosition = getOdometry();


     // Print the position for debugging
     Brain.Screen.printAt(5, 180, "X: %d, Y: %d, Heading: %d", currentPosition.x, currentPosition.y, currentPosition.heading);


     // Add a small delay to prevent flooding the screen with prints
     this_thread::sleep_for(100);
   }
   return 0;
}

void redLeft4() {
 drivePDC(-32,0);
 Clamp.set(true);
 turnPD(50);
 Intake.spin(forward);
 Hook.spin(forward);
 drivePDC(14,50);
 wait(.25,sec);
 turnPD(130);
 drivePDC(13,130);
 wait(.25,sec);
 drivePDC(-4,130);
 turnPD(100);
 wait(.25,sec);
 drivePDC(7,100);
 wait(.25,sec);
 drivePDC(-8,100);
 turnPD(215);
 wait(.25,sec);
 stupidRun(15);
 Clamp.set(false);
}

void blueRight4() {
 drivePDC(-32,0);
 Clamp.set(true);
 turnPD(-90);
 Intake.spin(forward);
 Hook.spin(forward);
 drivePDC(14,-90);
 wait(.25,sec);
 turnPD(-160);
 drivePDC(12,-160);
 wait(.25,sec);
 drivePDC(-4.5,-160);
 turnPD(-140);
 wait(.25,sec);
 drivePDC(5,-140);
 wait(.25,sec);
 drivePDC(-8,-140);
 turnPD(-255);
 wait(.25,sec);
 drivePDC(10,-140);
 Clamp.set(false);
 stupidRun(12.5);
 
}

void redRight2(){
 drivePDC(-30,0);
 Clamp.set(true);
 turnPD(-90);
 Intake.spin(forward);
 Hook.spin(forward);
 drivePDC(14,-90);
 wait(.25,sec);
 turnPD(150);
 drivePDC(15,150);
 Clamp.set(false);
 drivePDC(15,150);
}

void blueLeft2() {
 drivePDC(-30,0);
 Clamp.set(true);
 turnPD(90);
 Intake.spin(forward);
 Hook.spin(forward);
 drivePDC(14,90);
 wait(.25,sec);
 turnPD(-150);
 drivePDC(15,-150);
 Clamp.set(false);
 drivePDC(15,-150);
}

void redRightRush() {
  drivePDCLong(-40,0);
  turnPD(45);
  drivePDC(-9.25,45);
  Clamp.set(true);
  Intake.spin(forward);
  Hook.spin(forward);
  turnPD(-17.5);
  drivePDC(20,-17.5);
  turnPD(115);
  wait(1,sec);
  Intake.stop();
  Clamp.set(false);
  stupidRun(35);
}

void blueLeftRush() {
  drivePDCLong(-40,0);
  turnPD(-45);
  drivePDC(-9.25,-45);
  Clamp.set(true);
  Intake.spin(forward);
  Hook.spin(forward);
  turnPD(17.5);
  drivePDC(20,17.5);
  turnPD(-115);
  wait(1,sec);
  Intake.stop();
  Clamp.set(false);
  stupidRun(35);
}

void preload() {
  drivePDC(-32, 0);
  Clamp.set(true);
  Hook.spin(forward);
  wait(2,seconds);
  Clamp.set(false);
}
void selector(){
 switch(autonNum){
   case 1:
    redLeft4();
    break;
   case 2:
    redRight2();
    break;
   case 3:
    redRightRush();
    break;
   case 4:
    blueLeft2();
    break;
   case 5:
    blueRight4();
    break;
   case 6:
    blueLeftRush(); 
    break;
   case 7:
    preload(); 
    break;
   case 8:
   //WORST CASE SCENARIO ODOM & PID BREAK OFF LINE
    LeftFront.spin(forward);
    RightFront.spin(forward);
    LeftRear.spin(forward);
    RightRear.spin(forward);
    wait(1,seconds);
    LeftFront.stop(brake);
    RightFront.stop(brake);
    LeftRear.stop(brake);
    RightRear.stop(brake);
    break;
   default:
     wait(1,seconds);
     break;
 }
}

void skillsAuto() { 
 drivePDC(-4.5,0);
 Clamp.set(true);
 wait(.25,sec);
 turnPD(180);
 Intake.spin(forward);
 Hook.spin(forward);
 drivePDC(30,180);
 turnPD(270);
 drivePDC(24,270);
 turnPD(360);
 drivePDC(32,360);
 turnPD(240);
 drivePDC(7,240);
 turnPD(150);
 drivePDC(-7,150);
 wait(1.5,sec);
 Clamp.set(false);
 drivePDC(6,150);
 turnPD(270);
 wait(.25,sec);
 turnPD(270);
 Intake.stop();
 Hook.stop();
 drivePDCLong(-60,270);
 turnPD(310);
 drivePDC(-16,310);
 Clamp.set(true);

 Intake.spin(forward);
 Hook.spin(forward);
 turnPD(180);
 drivePDC(20,180);
 turnPD(90);
 drivePDC(24,90);
 turnPD(0);
 drivePDC(32,0);
 turnPD(130);
 drivePDC(8,130);
 turnPD(220);
 drivePDC(-5,220);
 wait(1.5,sec);
 Clamp.set(false);
}


void autonomous(void) {
 // ..........................................................................
 // Insert autonomous user code here.
 // ..........................................................................
 selector();
  
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


// Driver Control Functions
void LogDrive(){
 double LeftSpeed, RightSpeed;
 double LeftStick, RightStick;
 while (true)
 {


   if (Controller1.Axis1.value() >= 100)
   {
     RightStick = 75;
   }
   else if (Controller1.Axis1.value() <= -100)
   {
     RightStick = -75;
   }
   else
   {
     RightStick = Controller1.Axis1.value();
   }


   LeftSpeed = (fabs(RightStick) * RightStick / 127);
   RightSpeed = (fabs(Controller1.Axis3.position()) * Controller1.Axis3.position()) / 127;


   LeftFront.spin(forward, RightSpeed + LeftSpeed, percent);
   LeftMiddle.spin(forward, RightSpeed + LeftSpeed, percent);
   LeftRear.spin(forward, RightSpeed + LeftSpeed, percent);
   RightFront.spin(forward, RightSpeed - LeftSpeed, percent);
   RightMiddle.spin(forward, RightSpeed - LeftSpeed, percent);
   RightRear.spin(forward, RightSpeed - LeftSpeed, percent);
 }
}


void Controls(){
  while (true) {
    if (Controller1.ButtonL2.pressing()) {
      Intake.spin(reverse, 100, pct);
      Hook.spin(reverse,100,pct);
    }
    else if (Controller1.ButtonL1.pressing()) {
      Intake.spin(forward, 100, pct);
      Hook.spin(forward,100,pct);
    }
    else {
      Intake.stop(brake);
      Hook.stop(brake);
    }
   if (Controller1.ButtonR2.pressing() && Clamp.value() == false) {
     Clamp.set(true);
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(1, 1);
     Controller1.Screen.print("Clamp ON");
   }
   else if (Controller1.ButtonR1.pressing() && Clamp.value() == true) {
     Clamp.set(false);
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(1, 1);
     Controller1.Screen.print("Clamp OFF");
   }

    wait(10, msec);
  }
}


void usercontrol(void) {
 thread a (LogDrive);
 thread b (Controls);


 wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
 }


//
// Main will set up the competition functions and callbacks.
//
int main() {
 // Set up callbacks for autonomous and driver control periods.
 thread odometryThread(updateOdometry);
 thread brain(brainDisplay);


 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);


  // Run the pre-autonomous function.
 pre_auton();


 // Prevent main from exiting with an infinite loop.
 while (true) {
   wait(100, msec);
 }
}



