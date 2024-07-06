/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       jameshou                                                  */
/*    Created:      7/6/2024, 3:31:51 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

controller Controller1 = controller(primary);

// motor leftFrontMotor = motor(PORT19, ratio6_1, false);
// motor leftBackMotor = motor(PORT18, ratio6_1, false);
// motor rightFrontMotor = motor(PORT11, ratio6_1, true);
// motor rightBackMotor = motor(PORT14, ratio6_1, true);
// motor leftTopMotor = motor(PORT12, ratio6_1, true);
// motor rightTopMotor = motor(PORT15, ratio6_1, false);
// motor intakeMotor = motor(PORT20, ratio18_1, false);
// motor hookMotor = motor(PORT20, ratio18_1, false);
// motor liftMotor = motor(PORT16, ratio36_1, false);

digital_out clamp = digital_out(Brain.ThreeWirePort.A);

// inertial InertialA = inertial(PORT8);

void SplitDrive(){
    // leftFrontMotor.spin(reverse,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    // leftBackMotor.spin(reverse,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
    // leftTopMotor.spin(reverse,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    // rightFrontMotor.spin(reverse,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    // rightBackMotor.spin(reverse,Controller1.Axis3.position()-Controller1.Axis1.position(),pct);
    // rightTopMotor.spin(reverse,Controller1.Axis3.position()+Controller1.Axis1.position(),pct);
}

void intake() {
  if (Controller1.ButtonL1.pressing()){
    intakeMotor.spin(forward,100,pct);
  }
  else {
    intakeMotor.stop();
  }
}

void hook() {
  if (Controller1.ButtonL2.pressing()){
    hookMotor.spin(forward,100,pct);
  }
  else {
    hookMotor.stop();
  }
}

bool toggle_released = true;

void toggleClamp () {
  if(Controller1.ButtonX.pressing()) {
    if(toggle_released==false) {
      clamp.set(!clamp.value());
    }
    toggle_released=true;
  }
  if(!Controller1.ButtonX.pressing()) {
    toggle_released=false;
  }
}

void setDriveCoast() {
    leftFrontMotor.setStopping(coast);
    leftBackMotor.setStopping(coast);
    leftTopMotor.setStopping(coast);
    rightFrontMotor.setStopping(coast);
    rightBackMotor.setStopping(coast);
    rightTopMotor.setStopping(coast);
}

void setDriveBrake() {
    leftFrontMotor.setStopping(brake);
    leftBackMotor.setStopping(brake);
    leftTopMotor.setStopping(brake);
    rightFrontMotor.setStopping(brake);
    rightBackMotor.setStopping(brake);
    rightTopMotor.setStopping(brake);
}

void controllerDisplay(){
  while(true){
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("InertialA Rot: %f",InertialA.rotation(degrees));
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print("InertialA Head: %f",InertialA.heading(degrees));
    Controller1.Screen.setCursor(3,1);
    Controller1.Screen.print("LeftFrontMotor: %f",leftFrontMotor.position(degrees));
    // Allow other tasks to run
    this_thread::sleep_for(50);
  }
}

void initialize() {
    clamp.set(false);

    intakeMotor.setStopping(brake);
    hookMotor.setStopping(brake);

    setDriveBrake();

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("InertialA: Calibrating");

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("InertialA: Calibrating");

    InertialA.calibrate();
    // waits for the Inertial Sensor to calibrate
    while (InertialA.isCalibrating()) {
        wait(100, msec);
    }

    thread t1(controllerDisplay);
    // INSERT BRAIN SCREEN STUFF
}

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
  
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
    initialize();
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

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
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
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    splitDrive();
    intake();
    hook();
    toggleClamp();
    
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
