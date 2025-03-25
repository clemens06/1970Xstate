/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Clemens Pavlish                                           */
/*    Created:      3/13/2025, 12:55:16 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

brain Brain;

// A global instance of competition
competition Competition;

motor Left1 = motor(PORT2, ratio6_1, false);
motor Right1 = motor(PORT7, ratio6_1, true);
motor Left2 = motor(PORT5, ratio6_1, true);
motor Right2 = motor(PORT6, ratio6_1, false);
motor Left3 = motor(PORT12, ratio6_1, false);
motor Right3 = motor(PORT8, ratio6_1, true);

motor lb = motor(PORT10, ratio6_1, false);
motor Intake = motor(PORT11, ratio6_1, false);

controller Controller1 = controller(primary);

digital_out Clamp1 = digital_out(Brain.ThreeWirePort.H);
digital_out Clamp2 = digital_out(Brain.ThreeWirePort.G);

limit ClampBumper = limit(Brain.ThreeWirePort.A);

inertial LInertial = inertial(PORT3);
inertial RInertial = inertial (PORT4);

optical Optical = optical(PORT13);

rotation Rotation = rotation(PORT14);

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

  LInertial.calibrate();
  RInertial.calibrate();

}


float heading = 0.0;
float xpos = 0.0;
float ypos = 0.0;

//Odometry
void odom () {

  // Constants
  const float wheelDiameter = 2.75; // Diameter of the wheel in inches
  const float wheelCircumference = wheelDiameter * M_PI;
  const float trackWidth = 11.8; // Distance between the left and right wheels in inches

  // Variables to store the previous positions of the wheels
  static float prevLeftPos = 0;
  static float prevRightPos = 0;

  // Get the current positions of the wheels
  float currentLeftPos = (Left1.position(degrees) + Left2.position(degrees) + Left3.position(degrees)) / 3.0; 
  float currentRightPos = (Right1.position(degrees) + Right2.position(degrees) + Right3.position(degrees)) / 3.0;

  // Calculate the change in position of the wheels
  float deltaLeft = (currentLeftPos - prevLeftPos) / 360.0 * wheelCircumference;
  float deltaRight = (currentRightPos - prevRightPos) / 360.0 * wheelCircumference;

  // Update the previous positions of the wheels
  prevLeftPos = currentLeftPos;
  prevRightPos = currentRightPos;

  // Calculate the change in heading
  float deltaHeading = (deltaRight - deltaLeft) / trackWidth;
// Calculate the average distance traveled
float deltaDistance = (deltaLeft + deltaRight) / 2.0;

// Update the robot's position and heading
heading += deltaHeading;
xpos += deltaDistance * cos(heading);
ypos += deltaDistance * sin(heading);


}


void Straight(int x, int y){
 
  Right1.spin(forward, x*110, voltageUnits::mV);
  Left1.spin(forward, x*110, voltageUnits::mV);
  Left2.spin(forward, x*110, voltageUnits::mV);
  Right2.spin(forward, x*110, voltageUnits::mV);
  Left3.spin(forward, x*110, voltageUnits::mV);
  Right3.spin(forward, x*110, voltageUnits::mV);
  wait(y, msec);
  Right1.stop(hold);
  Left1.stop(hold);
  Left2.stop(hold);
  Right2.stop(hold);
  Left3.stop(hold);
  Right3.stop(hold);
 }
 
   //Reverse Function with Coast
 
   void AReverse(float x, float y){
 
  Right1.spin(reverse, x*110, voltageUnits::mV);
  Left1.spin(reverse, x*110, voltageUnits::mV);
  Left2.spin(reverse, x*110, voltageUnits::mV);
  Right2.spin(reverse, x*110, voltageUnits::mV);
  Right3.spin(reverse, x*110, voltageUnits::mV);
  Left3.spin(reverse, x*110, voltageUnits::mV);
  wait(y, msec);
  Right1.stop(coast);
  Right2.stop(coast);
  Right3.stop(coast);
  Left1.stop(coast);
  Left2.stop(coast);
  Left3.stop(coast);
   }
 
   //Reverse function with hold stop
 
   void Reverse(float x, float y){
 
  Right1.spin(reverse, x*110, voltageUnits::mV);
  Left1.spin(reverse, x*110, voltageUnits::mV);
  Left2.spin(reverse, x*110, voltageUnits::mV);
  Right2.spin(reverse, x*110, voltageUnits::mV);
  Right3.spin(reverse, x*110, voltageUnits::mV);
  Left3.spin(reverse, x*110, voltageUnits::mV);
  wait(y, msec);
  Right1.stop(hold);
  Right2.stop(hold);
  Right3.stop(hold);
  Left1.stop(hold);
  Left2.stop(hold);
  Left3.stop(hold);
   }
 
   //Left turn function
 
   void leftTurn(float x, float y){
 
  Right1.spin(forward, x*110, voltageUnits::mV);
  Right2.spin(forward, x*110, voltageUnits::mV);
  Right3.spin(forward, x*110, voltageUnits::mV);
  Left1.spin(reverse, x*110, voltageUnits::mV);
  Left2.spin(reverse, x*110, voltageUnits::mV);
  Left3.spin(reverse, x*110, voltageUnits::mV);
  wait(y, msec);
  Right1.stop(hold);
  Right2.stop(hold);
  Right3.stop(hold);
  Left1.stop(hold);
  Left2.stop(hold);
  Left3.stop(hold);
 
 
 }
 
 //Right turn function
 
 void rightTurn(float x, float y){
 
  Right1.spin(reverse, x*110, voltageUnits::mV);
  Right2.spin(reverse, x*110, voltageUnits::mV);
  Right3.spin(reverse, x*110, voltageUnits::mV);
  Left1.spin(forward, x*110, voltageUnits::mV);
  Left2.spin(forward, x*110, voltageUnits::mV);
  Left3.spin(forward, x*110, voltageUnits::mV);
  wait(y, msec);
  Right1.stop(hold);
  Right2.stop(hold);
  Right3.stop(hold);
  Left1.stop(hold);
  Left2.stop(hold);
  Left3.stop(hold);
 
 }

//Color Sort

int colorSortSetting = 0;

void colorSort() {

  Optical.integrationTime(1);
  Optical.objectDetectThreshold(100);
  float hue = Optical.hue();

  if (colorSortSetting == 0) {

    if ((hue>355 && hue <360) || (hue>0 && hue<15)) {
    
      float currentIntakePlace = Intake.position(degrees);
      float launchPlace = currentIntakePlace - 500;
      
      while (Intake.position(degrees) < launchPlace) {
        Intake.spin(forward, 100, velocityUnits::pct);
        wait(300, msec);
      }
    }
    
  } 
  
  if (colorSortSetting == 1) {

    if ((hue<290 && hue>240)) {
      
      float currentIntakePlace = Intake.position(degrees);
      float launchPlace = currentIntakePlace + 900;
      
      while (Intake.position(degrees) < launchPlace) {
        
        
      if (Controller1.ButtonR1.pressing()) {
        Intake.spin(forward, 100, velocityUnits::pct);
      } else if (Controller1.ButtonR2.pressing()) {
        Intake.spin(reverse, 100, velocityUnits::pct);
      } else {
        Intake.stop(hold);
      }
        wait(20, msec);
      }
      while (Intake.position(degrees) >= launchPlace) {
        Intake.stop();
      }
    }
  } 
  if (colorSortSetting == 2) {
  }
}

void colorSortChanger() {
  if (Controller1.ButtonRight.pressing()) {
    colorSortSetting = (colorSortSetting + 1) % 3; // Cycle through 0, 1, 2
  } else if (Controller1.ButtonLeft.pressing()) {
    colorSortSetting = (colorSortSetting - 1) % 3; // Cycle through 2, 1, 0
  }
}


 // Lateral PID
int drivePID(int driveDistance, int maxPower) {

  if (ClampBumper.pressing()) {

  float kP = 0.00115;
  float kI = 0.0;
  float kD = 0.014;
  float error = 0.0;
  float integral = 0.0;
  float derivative = 0.0;
  float prevError = 0.0;
  float motorPower = 0.0;
  float prevMotorPower = 0.0;

  Right1.setPosition(0, degrees);
  Right2.setPosition(0, degrees);
  Right3.setPosition(0, degrees);
  Left1.setPosition(0, degrees);
  Left2.setPosition(0, degrees);
  Left3.setPosition(0, degrees);

  while(true){

    float currentDistance = (Left1.position(degrees) + Left2.position(degrees) + Left3.position(degrees) + Right1.position(degrees) + Right2.position(degrees) +Right3.position(degrees)) / 6.0;

    //calculate error
    error = driveDistance - currentDistance; 

   //updated the integral term if |error| < 200
    if (error < 100 && error > -100) {
    
    integral += error; 
    
    }

    //find derivative
    derivative = error - prevError; 

    //determine motor power
    motorPower = (kP * error) + (kI * integral) + (kD * derivative); 

    //1028T stuff 
    /*
    if (motorPower > spd){

      motorPower = spd;
    }
    */

    //Restrict motor power to within + or - 100%
    if (motorPower > 1) motorPower = 1;
    if (motorPower < -1) motorPower = -1;
    motorPower = motorPower *11;
    
    
    //Acceleration limiter
    float accelRate = 0.15f;
    if (motorPower > prevMotorPower + accelRate) motorPower = prevMotorPower + accelRate;
    if (motorPower < prevMotorPower - accelRate) motorPower = prevMotorPower - accelRate;
    

    //Go
    Left1.spin(forward, maxPower * motorPower, voltageUnits::volt);
    Left2.spin(forward, maxPower * motorPower, voltageUnits::volt);
    Left3.spin(forward, maxPower * motorPower, voltageUnits::volt);
    Right1.spin(forward, maxPower * motorPower, voltageUnits::volt);
    Right2.spin(forward, maxPower * motorPower, voltageUnits::volt);
    Right3.spin(forward, maxPower * motorPower, voltageUnits::volt);

    
    //Stop controller when near target
    if (error > -10 && error < 10 && error - prevError > -10 && error - prevError < 10) {

    break;

    }

    //update prev
    prevMotorPower = motorPower;
    prevError = error;

    //save CPU
    wait (20, msec);
  }

  //stop
    Left1.stop();
    Left2.stop();
    Left3.stop();
    Right1.stop();
    Right2.stop();
    Right3.stop();

  return 1;
}

else {


  float kP = 0.00115;
  float kI = 0.0;
  float kD = 0.014;
  float error = 0.0;
  float integral = 0.0;
  float derivative = 0.0;
  float prevError = 0.0;
  float motorPower = 0.0;
  float prevMotorPower = 0.0;

  Right1.setPosition(0, degrees);
  Right2.setPosition(0, degrees);
  Right3.setPosition(0, degrees);
  Left1.setPosition(0, degrees);
  Left2.setPosition(0, degrees);
  Left3.setPosition(0, degrees);

  while(true){

    float currentDistance = (Left1.position(degrees) + Left2.position(degrees) + Left3.position(degrees) + Right1.position(degrees) + Right2.position(degrees) +Right3.position(degrees)) / 6.0;

    //calculate error
    error = driveDistance - currentDistance; 

   //updated the integral term if |error| < 200
    if (error < 100 && error > -100) {
    
    integral += error; 
    
    }

    //find derivative
    derivative = error - prevError; 

    //determine motor power
    motorPower = (kP * error) + (kI * integral) + (kD * derivative); 

    //Restrict motor power to within + or - 100%
    if (motorPower > 1) motorPower = 1;
    if (motorPower < -1) motorPower = -1;
    motorPower = motorPower *11;
    
    
    //Acceleration limiter
    float accelRate = 0.2f;
    if (motorPower > prevMotorPower + accelRate) motorPower = prevMotorPower + accelRate;
    if (motorPower < prevMotorPower - accelRate) motorPower = prevMotorPower - accelRate;
    

    //Go
    Left1.spin(forward, 8 * motorPower, voltageUnits::volt);
    Left2.spin(forward, 8 * motorPower, voltageUnits::volt);
    Left3.spin(forward, 8 * motorPower, voltageUnits::volt);
    Right1.spin(forward, 8 * motorPower, voltageUnits::volt);
    Right2.spin(forward, 8 * motorPower, voltageUnits::volt);
    Right3.spin(forward, 8 * motorPower, voltageUnits::volt);

    
    //Stop controller when near target
    if (error > -10 && error < 10 && error - prevError > -10 && error - prevError < 10) {

    break;

    }

    //update prev
    prevMotorPower = motorPower;
    prevError = error;

    //save CPU
    wait (20, msec);
  }

  //stop
    Left1.stop();
    Left2.stop();
    Left3.stop();
    Right1.stop();
    Right2.stop();
    Right3.stop();

  return 1;

}


}

int turnPID(int turnDistance) {

  float kP = 0.004;
  float kI = 0.00;
  float kD = 0.05;
  float error = 0.0;
  float integral = 0.0;
  float derivative = 0.0;
  float prevError = 0.0;
  float motorPower = 0.0;
  float prevMotorPower = 0.0;
  float startDistance = (RInertial.rotation(degrees) + LInertial.rotation(degrees)) / 2.0;  

 while(true) {
  
    float currentDistance = startDistance - (RInertial.rotation(degrees) + LInertial.rotation(degrees)) / 2.0;
   
    //calculate error
    error = turnDistance - currentDistance; 

    //updated the integral term if |error| < 200
    if (error < 100 && error > -100) {
    
    integral += error; 
    
    }

    //find derivative
    derivative = error - prevError; 

    //determine motor power
    motorPower = (kP * error) + (kI * integral) + (kD * derivative); 

    //Restrict motor power to within + or - 100%
    if (motorPower > 1) motorPower = 1;
    if (motorPower < -1) motorPower = -1;
    motorPower = motorPower *11;
    
    
    //Acceleration limiter
    float accelRate = 0.25f;
    if (motorPower > prevMotorPower + accelRate) motorPower = prevMotorPower + accelRate;
    if (motorPower < prevMotorPower - accelRate) motorPower = prevMotorPower - accelRate;
    
    Left1.spin(forward, -9 * motorPower, voltageUnits::volt);
    Left2.spin(forward, -9 * motorPower, voltageUnits::volt);
    Left3.spin(forward, -9 * motorPower, voltageUnits::volt);
    Right1.spin(forward, 9 * motorPower, voltageUnits::volt);
    Right2.spin(forward, 9 * motorPower, voltageUnits::volt);
    Right3.spin(forward, 9 * motorPower, voltageUnits::volt);

    //Exit in certain range
    if (error < 2.5 && error > -2.5) {
    
    break;
    
    }

    //update prev
    prevMotorPower = motorPower;
    prevError = error;

  wait(20, msec);
}

//stop
Left1.stop();
Left2.stop();
Left3.stop();
Right1.stop();
Right2.stop();
Right3.stop();

  return 1;
}

//Ladybrown PID
 

 int lbPID(int driveDistance) {

    float kP = 0.02;
    float kI = 0.0;
    float kD = 0.35;
    float error = 0.0;
    float integral = 0.0;
    float derivative = 0.0;
    float prevError = 0.0;
    float motorPower = 0.0;
    float prevMotorPower = 0.0;
    lb.setStopping(hold);
    
  
    while(true){
  
      float currentDistance = Rotation.position(degrees);
  
      //calculate error
      error = driveDistance - currentDistance; 

      //find integral
      integral += error; 
  
      //find derivative
      derivative = error - prevError; 
  
      //determine motor power
      motorPower = (kP * error) + (kI * integral) + (kD * derivative); 
  
      //Restrict motor power to within + or - 100%
      if (motorPower > 1) motorPower = 1;
      if (motorPower < -1) motorPower = -1;
      motorPower = motorPower *11;
  
     /* //Acceleration limiter
      float accelRate = 0.4f;
      if (motorPower > prevMotorPower + accelRate) motorPower = prevMotorPower + accelRate;
      if (motorPower < prevMotorPower - accelRate) motorPower = prevMotorPower - accelRate;
*/

      //Go
      lb.spin(reverse, 5 * motorPower, voltageUnits::volt);
  
      
      //Stop controller when near target
      if (error > -2 && error < 2 && error - prevError > -2 && error - prevError < 2) {
  
      break;
  
      }
  
      //update prev
      prevMotorPower = motorPower;
      prevError = error;
  
      //save CPU
      wait (20, msec);
    }
  
    //stop
      lb.stop();
  
    return 1;
  }

//ladybrown positions

void lbMovement() {
  //for stowed
if (Rotation.position(degrees) < 20 && Rotation.position(degrees) >= 0) {
  if (Controller1.ButtonUp.pressing()){
    lbPID(28);
  }

}
//for ready
else if (Rotation.position(degrees) <90  && Rotation.position(degrees) > 20) {
  if (Controller1.ButtonUp.pressing()) {
    lbPID(113);
  }
  else if (Controller1.ButtonDown.pressing()){
    lbPID(4);
  }
}
//for scoring
else if (Rotation.position(degrees) > 90 && Rotation.position(degrees) < 170) {
  if (Controller1.ButtonDown.pressing()) {
    lbPID(28);
  }
  else if (Controller1.ButtonUp.pressing()) {
    lbPID(193);
  }
}
//for alliance
else if (Rotation.position(degrees) > 170 && Rotation.position(degrees) < 210) {
  if (Controller1.ButtonDown.pressing()) {
    lbPID(240);
  }
  else if (Controller1.ButtonUp.pressing()) {
    lbPID(113);
  }
}
//for down
else if (Rotation.position(degrees) > 210 && Rotation.position(degrees) < 300) {
  if (Controller1.ButtonDown.pressing()) {
    lbPID(193);
  }
}
}


bool intakeRunning = false;

// Function to run the intake
int runIntake() {
  while (intakeRunning) {
    Intake.spin(reverse, 100, velocityUnits::pct);
    wait(20, msec);
  }
  Intake.stop();
  return 0;
}

//task for printing info
bool printingStuff = false;
int printInfo() {
  while (printingStuff) {
    Brain.Screen.printAt(10, 50, "Left Inertial: %f", LInertial.rotation(degrees));
    Brain.Screen.printAt(10, 70, "Right Inertial: %f", RInertial.rotation(degrees));
    Brain.Screen.printAt(10, 90, "Inertial Heading: %f", (RInertial.rotation(degrees) + LInertial.rotation(degrees)) / 2.0);
    Brain.Screen.printAt(10,170, "LB Position: %f", Rotation.angle(degrees));
    Brain.Screen.printAt(10, 110, "Heading: %f", heading);
    Brain.Screen.printAt(10, 130, "X: %f", xpos);
    Brain.Screen.printAt(10, 150, "Y: %f", ypos);
    Brain.Screen.printAt(10, 190, "Color Sort Setting: %f", colorSortSetting);

 if (colorSortSetting == 0) {
   Controller1.Screen.print("Color Sort: Red");
 } else if (colorSortSetting == 1) {
   Controller1.Screen.print("Color Sort: Blue");
 } else if (colorSortSetting == 2) {
   Controller1.Screen.print("Color Sort: Off");
 }
    wait(20, msec);
  }
  return 0;
}

void autonomous(void) {

  printingStuff = true;
  task printTask(printInfo);
  Controller1.Screen.clearScreen();

  if (ClampBumper.pressing()) {

    Clamp1.set(true);
    Clamp2.set(true);

  }

 //red neg
  lbPID(193);
  wait(300, msec); 
  intakeRunning = true;
  task intakeTask(runIntake);
  drivePID(-420, 11);
  lbPID(3);
  turnPID(52);
  drivePID(-1050, 1);
  wait(100, msec);
  Clamp1.set(true);
  Clamp2.set(true);
  wait(200, msec);
  turnPID(110);
  drivePID(1000, 8);
  turnPID(120);
  intakeRunning = false;
  intakeTask.stop();
  drivePID(-1200, 11);


}


void usercontrol(void) {


  Optical.setLight(ledState::on);
  Controller1.Screen.clearScreen();
  Intake.setStopping(hold);

  while (1) {

    colorSortChanger();
    //colorSort();
    odom();
    lbMovement();

    colorSortSetting = 1;

    Brain.Screen.printAt(10, 50, "Left Inertial: %f", LInertial.rotation(degrees));
    Brain.Screen.printAt(10, 70, "Right Inertial: %f", RInertial.rotation(degrees));
    Brain.Screen.printAt(10, 90, "Inertial Heading: %f", (RInertial.rotation(degrees) + LInertial.rotation(degrees)) / 2.0);
    Brain.Screen.printAt(10, 110, "Heading: %f", heading);
    Brain.Screen.printAt(10, 130, "X: %f", xpos);
    Brain.Screen.printAt(10, 150, "Y: %f", ypos);
    Brain.Screen.printAt(10,170, "LB Position: %f", Rotation.angle(degrees));
    Brain.Screen.printAt(10, 190, "Current Hue: %f", Optical.hue());
    Brain.Screen.printAt(10, 210, "Color Sort: %f", colorSortSetting);
   
 if (colorSortSetting == 0) {
  Controller1.Screen.print("Color Sort: Red");
} else if (colorSortSetting == 1) {
  Controller1.Screen.print("Color Sort: Blue");
} else if (colorSortSetting == 2) {
  Controller1.Screen.print("Color Sort: Off");
}

    int check = ClampBumper.pressing();
    int lDrive = (Controller1.Axis3.position() + Controller1.Axis4.position());
    int rDrive = (Controller1.Axis3.position() - Controller1.Axis4.position());
  
     if(lDrive != 0 || rDrive != 0){
     Right1.spin(forward, rDrive*110, voltageUnits::mV);
     Right2.spin(forward, rDrive*110, voltageUnits::mV);
     Right3.spin(forward, rDrive*110, voltageUnits::mV);
     Left1.spin(forward, lDrive*110, voltageUnits::mV);
     Left2.spin(forward, lDrive*110, voltageUnits::mV);
     Left3.spin(forward, lDrive*110, voltageUnits::mV);
     }else{
     Right1.stop(hold);
     Right2.stop(hold);
     Right3.stop(hold);
     Left1.stop(hold);
     Left2.stop(hold);
     Left3.stop(hold);
     }

     if (Controller1.ButtonR1.pressing()) {
      Intake.spin(forward, 100, velocityUnits::pct);
    } else if (Controller1.ButtonR2.pressing()) {
      Intake.spin(reverse, 100, velocityUnits::pct);
    } else {
      Intake.stop(hold);
    }

    if (Controller1.ButtonA.pressing()) {

      Intake.spin(forward, 20, velocityUnits::pct);

    }

    if (ClampBumper.pressing()) {

      Clamp1.set(true);
      Clamp2.set(true);

    }


    
  if (Controller1.ButtonX.pressing()) {
    Clamp1.set(false);
    Clamp2.set(false);
   }else if(Controller1.ButtonB.pressing()){
   Clamp1.set(true);
   Clamp2.set(true);
   }
  if (Controller1.ButtonL1.pressing()) {
    lb.spin(forward, 11000, voltageUnits::mV);
  } else if (Controller1.ButtonL2.pressing()) {
    lb.spin(reverse, 11000, voltageUnits::mV);
  } else {
    lbMovement();
    lb.stop(hold);
  } 
  }
  wait(20, msec);
  }

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  LInertial.calibrate();
  RInertial.calibrate();
  Clamp1.set(false);
  Clamp2.set(false);
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}