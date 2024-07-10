/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       andrew                                                    */
/*    Created:      6/28/2024, 1:13:14 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;


// A global instance of competition
competition Competition;

brain Brain;

controller Controller1=controller(primary);
motor LeftFront=motor(PORT1, ratio36_1, true);
motor RightFront=motor(PORT2, ratio36_1, false);
motor LeftMiddle=motor(PORT3, ratio36_1, true);
motor RightMiddle=motor(PORT4, ratio36_1, false);
motor LeftRear=motor(PORT5, ratio36_1, true);
motor RightRear=motor(PORT6, ratio36_1, false);
motor Intake=motor(PORT7, ratio18_1, false);
motor Hook=motor(PORT8, ratio18_1, true);
motor Arm=motor(PORT9, ratio18_1, false);
digital_out Clamp=digital_out(Brain.ThreeWirePort.A);
inertial Gyro(PORT10);
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

void pre_auton(void) {
  RightFront.setStopping(coast);
  RightRear.setStopping(coast);
  RightMiddle.setStopping(coast);
  LeftFront.setStopping(coast);
  LeftRear.setStopping(coast);
  LeftMiddle.setStopping(coast);
  Intake.setVelocity(100, percent);
  Arm.setVelocity(75,percent);
  Clamp.set(false);
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

double x = 0.0, y = 0.0, theta = 0.0;
const double WHEEL_DIAMETER = 3.25; // Example wheel diameter in inches
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
const double TICKS_PER_REV = 360.0; // Encoder ticks per wheel revolution
const double WHEEL_TRACK = 15; // Distance between left and right wheels in inches

void updateOdometry() {
    double leftPosition = (LeftRear.position(degrees)+LeftMiddle.position(degrees)+LeftFront.position(degrees))/3;
    double rightPosition = (RightRear.position(degrees)+RightMiddle.position(degrees)+RightFront.position(degrees))/3;
    
    double leftDistance = (leftPosition / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
    double rightDistance = (rightPosition / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
    
    double distance = (leftDistance + rightDistance) / 2.0;
    double deltaTheta = (rightDistance - leftDistance) / WHEEL_TRACK;
    
    theta += deltaTheta;
    x += distance * cos(theta);
    y += distance * sin(theta);
}

void drivePID(double target, bool isTurn = false) {
    LeftMiddle.resetPosition();
    RightMiddle.resetPosition();
    LeftFront.resetPosition();
    RightFront.resetPosition();
    LeftRear.resetPosition();
    RightRear.resetPosition();
    double integral = 0;
    double derivative;
    double kP = 0.75;
    double kI = 0.1; // Integral gain, adjust as needed
    double kD = 0.4;
    double previousError = target;
    double maxSpeed = 100; // Maximum speed percentage
    double rampUpDistance = target * 0.25; // 25% of the distance to ramp up
    double rampDownDistance = target * 0.75; // 75% of the distance to start ramping down

    while (fabs(previousError) > 3) {
        updateOdometry();

        int currentPosition = ((LeftRear.position(degrees)+LeftMiddle.position(degrees)+LeftFront.position(degrees)) + (RightRear.position(degrees)+RightMiddle.position(degrees)+RightFront.position(degrees))) / 6;
        double error = target - currentPosition;
        integral += error;
        derivative = error - previousError;

        // Speed profile calculation
        double speedFactor;
        if (currentPosition < rampUpDistance) {
            // Ramp up
            speedFactor = (double)currentPosition / rampUpDistance;
        } else if (currentPosition > rampDownDistance) {
            // Ramp down
            speedFactor = (double)(target - currentPosition) / (target - rampDownDistance);
        } else {
            // Constant speed
            speedFactor = 1.0;
        }

        // Clamp speedFactor to be within 0 and 1
        speedFactor = fmin(fmax(speedFactor, 0), 1);

        // PID output with speed profile
        double output = (error * kP + integral * kI + derivative * kD) * speedFactor * maxSpeed;
        
        // Adjust for turning
        if (isTurn) {
            LeftRear.spin(forward, output, pct);
            RightRear.spin(reverse, output, pct);
            LeftMiddle.spin(forward, output, pct);
            RightMiddle.spin(reverse, output, pct);
            LeftFront.spin(forward, output, pct);
            RightFront.spin(reverse, output, pct);
        } else {
            LeftRear.spin(forward, output, pct);
            RightRear.spin(forward, output, pct);
            LeftMiddle.spin(forward, output, pct);
            RightMiddle.spin(forward, output, pct);
            LeftFront.spin(forward, output, pct);
            RightFront.spin(forward, output, pct);
        }

        previousError = error;
        wait(10, msec);
    }
    LeftRear.stop();
    RightRear.stop();
    LeftMiddle.stop();
    RightMiddle.stop();
    LeftFront.stop();
    RightFront.stop();
}

void turnToAngle(double targetAngle) {
    double currentAngle = Gyro.rotation(degrees);
    double deltaAngle = targetAngle - currentAngle;

    // Normalize deltaAngle to be between -180 and 180 degrees
    while (deltaAngle > 180.0) {
        deltaAngle -= 360.0;
    }
    while (deltaAngle < -180.0) {
        deltaAngle += 360.0;
    }

    // Use PID control to turn to the target angle
    drivePID(deltaAngle, true);
}
/*
void driveToPoint(double targetX, double targetY) {
    // Calculate distance and angle to target point
    double deltaX = targetX - x;
    double deltaY = targetY - y;
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    double targetTheta = atan2(deltaY, deltaX);

    // Turn to the target angle using inertial sensor
    turnToAngle(targetTheta);

    // Calculate arc parameters
    double radius = 24.0; // Example radius for the arc (adjust as needed)
    double arcCenterX = x + radius * cos(theta + M_PI / 2.0);
    double arcCenterY = y + radius * sin(theta + M_PI / 2.0);
    double startAngle = atan2(y - arcCenterY, x - arcCenterX);
    double endAngle = atan2(targetY - arcCenterY, targetX - arcCenterX);
    double arcAngle = endAngle - startAngle;

    // Drive along the arc
    double arcLength = radius * arcAngle;
    drivePID(arcLength);

    // Drive straight to the target point
    drivePID(distance);
}
*/
/*
void driveToPoint(double targetX, double targetY) {
    // Calculate distance and angle to target point
    double deltaX = targetX - x;
    double deltaY = targetY - y;
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    double targetTheta = atan2(deltaY, deltaX);

    // Turn to the target angle using inertial sensor
    turnToAngle(targetTheta);

    // Calculate arc parameters for inward curve
    double radius = 12.0; // Example radius for the arc (adjust as needed)
    double arcCenterX = x - radius * cos(theta + M_PI / 2.0);
    double arcCenterY = y - radius * sin(theta + M_PI / 2.0);
    double startAngle = atan2(y - arcCenterY, x - arcCenterX);
    double endAngle = atan2(targetY - arcCenterY, targetX - arcCenterX);
    double arcAngle = endAngle - startAngle;

    // Handle cases where arcAngle crosses -π/π boundary
    if (arcAngle > M_PI) {
        arcAngle -= 2 * M_PI;
    } else if (arcAngle < -M_PI) {
        arcAngle += 2 * M_PI;
    }

    // Drive along the arc
    double arcLength = fabs(radius * arcAngle); // Use absolute value for length
    drivePID(arcLength);

    // Drive straight to the target point
    drivePID(distance);
}
*/

void driveToPoint(double targetX, double targetY) {
    const double kP_pos = 0.5; // Proportional gain for position correction
    const double kP_ang = 0.5; // Proportional gain for angle correction
    const double kP_arc = 0.8; // Proportional gain for arc movement

    // Calculate distance and angle to target point
    double deltaX = targetX - x;
    double deltaY = targetY - y;
    double distance = sqrt(deltaX * deltaX + deltaY * deltaY);
    double targetTheta = atan2(deltaY, deltaX);

    double maxSpeed = 100;
    // Determine if arc movement is needed
    if (distance > 10.0) {
        // Large enough distance to use arc movement (adjust threshold as needed)
        double arcRadius = 24.0; // Radius in inches, adjust as needed

        // Calculate arc parameters
        double arcCenterX = x - arcRadius * cos(theta + M_PI / 2.0);
        double arcCenterY = y - arcRadius * sin(theta + M_PI / 2.0);
        double startAngle = atan2(y - arcCenterY, x - arcCenterX);
        double endAngle = atan2(targetY - arcCenterY, targetX - arcCenterX);
        double arcAngle = endAngle - startAngle;

        // Handle cases where arcAngle crosses -π/π boundary
        if (arcAngle > M_PI) {
            arcAngle -= 2 * M_PI;
        } else if (arcAngle < -M_PI) {
            arcAngle += 2 * M_PI;
        }

        // Drive along the arc
        while (true) {
            updateOdometry();

            double currentArcLength = fabs(arcRadius * (theta + M_PI / 2.0));
            double error = fabs(arcRadius * arcAngle) - currentArcLength;
            double arcSpeed = error * kP_arc;

            // Adjust speeds for clockwise or counterclockwise movement
            double leftSpeed = (arcRadius > 0) ? arcSpeed : -arcSpeed;
            double rightSpeed = (arcRadius > 0) ? -arcSpeed : arcSpeed;

            // Clamp speeds to be within maxSpeed
            leftSpeed = fmin(fmax(leftSpeed, -maxSpeed), maxSpeed);
            rightSpeed = fmin(fmax(rightSpeed, -maxSpeed), maxSpeed);

            LeftRear.spin(forward, leftSpeed, pct);
            RightRear.spin(forward, rightSpeed, pct);
            LeftMiddle.spin(forward, leftSpeed, pct);
            RightMiddle.spin(forward, rightSpeed, pct);
            LeftFront.spin(forward, leftSpeed, pct);
            RightFront.spin(forward, rightSpeed, pct);

            if (fabs(error) < 1.0) {
                break; // Exit loop when arc is completed
            }

            wait(10, msec);
        }

        // Stop the robot briefly to ensure smooth transition into straight-line movement
        LeftRear.stop();
        RightRear.stop();
        LeftMiddle.stop();
        RightMiddle.stop();
        LeftFront.stop();
        RightFront.stop();
    }

    // Continue with straight-line PID movement towards the target point
    while (true) {
        updateOdometry();
        
        // Recalculate distance and angle to target point
        deltaX = targetX - x;
        deltaY = targetY - y;
        distance = sqrt(deltaX * deltaX + deltaY * deltaY);
        targetTheta = atan2(deltaY, deltaX);
        
        if (distance < 3.0) {
            break;
        }

        double angleError = targetTheta - theta;
        
        // Normalize angleError to be between -π and π
        while (angleError > M_PI) {
            angleError -= 2 * M_PI;
        }
        while (angleError < -M_PI) {
            angleError += 2 * M_PI;
        }

        // PID control for position
        double posCorrection = distance * kP_pos;

        // PID control for angle
        double angCorrection = angleError * kP_ang;

        // Calculate left and right motor speeds
        double leftSpeed = posCorrection - angCorrection;
        double rightSpeed = posCorrection + angCorrection;

        // Clamp speeds to be within maxSpeed
        leftSpeed = fmin(fmax(leftSpeed, -maxSpeed), maxSpeed);
        rightSpeed = fmin(fmax(rightSpeed, -maxSpeed), maxSpeed);

        LeftRear.spin(forward, leftSpeed, pct);
        RightRear.spin(forward, rightSpeed, pct);
        LeftMiddle.spin(forward, leftSpeed, pct);
        RightMiddle.spin(forward, rightSpeed, pct);
        LeftFront.spin(forward, leftSpeed, pct);
        RightFront.spin(forward, rightSpeed, pct);

        wait(10, msec);
    }

    LeftRear.stop();
    RightRear.stop();
    LeftMiddle.stop();
    RightMiddle.stop();
    LeftFront.stop();
    RightFront.stop();
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  // Move forward 100 units
    drivePID(100);

    // Immediately turn by a certain angle without slowing down
    // Example: turn 90 degrees, adjust 'turnAngle' as needed
    drivePID(90, true);


    driveToPoint(0, 0);     // Move to (0, 0)
    driveToPoint(0, 144);   // Move to (0, 12 feet)
    driveToPoint(144, 144); // Move to (12 feet, 12 feet)
    driveToPoint(144, 0);   // Move to (12 feet, 0)
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    LeftFront.spin(forward, Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    LeftRear.spin(forward, Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    LeftMiddle.spin(forward, Controller1.Axis3.position()+Controller1.Axis1.position(),pct);

    RightFront.spin(forward, Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    RightRear.spin(forward, Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    RightMiddle.spin(forward, Controller1.Axis3.position()-Controller1.Axis1.position(),pct);


    if(Controller1.ButtonL1.pressing()){
      Intake.spin(forward);
      Hook.spin(forward);
    }
    if(Controller1.ButtonL2.pressing()){
      Intake.spin(reverse);
      Hook.spin(reverse);
    }
    if((Controller1.ButtonL1.pressing()==false)&&(Controller1.ButtonL2.pressing()==false)){
      Intake.stop();
      Hook.stop();
    }
    

    if(Controller1.ButtonR1.pressing()){
      Arm.spin(forward);
    }
    if(Controller1.ButtonR1.pressing()==false){
      Arm.stop();
    }

    if(Controller1.ButtonR2.pressing()){
      Arm.spin(reverse);
    }
    if(Controller1.ButtonR2.pressing()==false){
      Arm.stop();
    }

    if(Controller1.ButtonUp.pressing()){
      while(Controller1.ButtonUp.pressing()){
        wait(10, msec);
      }
      Clamp.set(false);
     
    }
    if(Controller1.ButtonDown.pressing()){
      while(Controller1.ButtonDown.pressing()){
        wait(10, msec);
      }
      Clamp.set(true);
     
    }




    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Gyro.calibrate();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
