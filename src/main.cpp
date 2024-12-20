#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;

int MaryResetTime = 400; // Time to wait for Mary to reset.
int MaryPrimeTime = 100; // Time to wait from reset to primed for picking up ring.
int MaryScoreTime = 400; // Time to wait for Mary to score.

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.




// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// Include the V5 Library
#include "vex.h"
  
// Allows for easier use of the VEX Library
using namespace vex;

digital_out Mogo = digital_out(Brain.ThreeWirePort.H);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G);

controller Controller1 = controller(primary);
motor L1 = motor(PORT4, ratio6_1, true);

motor L2 = motor(PORT5, ratio6_1, true);

motor L3 = motor(PORT6, ratio6_1, true);

motor R1 = motor(PORT1, ratio6_1, false);

motor R2 = motor(PORT2, ratio6_1, false);

motor R3 = motor(PORT3, ratio6_1, false);

inertial Inert = inertial(PORT10);

motor Intake = motor(PORT20, ratio6_1, false);

motor55 MaryLeft = motor55(PORT12);
motor55 MaryRight = motor55(PORT11);
int MaryState = 0;

motor_group DriveL = motor_group(L1, L2, L3);
motor_group DriveR = motor_group(R1, R2, R3);

const double drivekp{ 1.1 };
const double driveki{ 0.0 };
const double drivekd{ 10 };
const double turnkp{ 0.35 };
const double turnki{ 0.03 };
const double turnkd{ 3.0 };
const double swingkp{ 0.4 };
const double swingki{ 0.015 };
const double swingkd{ 1.7 };

const int driving_gear{ 48 };
const int driven_gear{ 72 };
const double wheel_diameter{ 3.25 };

bool redirect = false;

float deadband(float input, float width){
  if (fabs(input)<width){
    return(0);
  }
  return(input);
}

//PID Class + util

class PID {
  public:
    //PID variables
    double piderror{ 0 };
    //PID constants
    double kp{ 0 };
    double ki{ 0 };
    double kd{ 0 };
    //value at which the integral term activates
    double starti{ 0 };
    //error value at which the robot is considered settled
    double settle_error{ 0 };
    //settle time at which the robot exits the loop
    double settle_time{ 0 };
    //total runtime at which the robot exits the loop
    double timeout{ 0 };
    //total area under error vs. time curve (e(t))
    double integral{ 0 };
    //error value at previous loop iteration
    double previous_error{ 0 };
    //PID loop output (u(t))
    double output{ 0 };
    //PID timers
    double time_spent_settled{ 0 };
    double time_spent_running{ 0 };
 
    PID(double error, double kp, double ki, double kd, double starti, double settle_error, double settle_time, double timeout) :
      //main class constructor
      piderror(error),
      kp(kp),
      ki(ki),
      kd(kd),
      starti(starti),
      settle_error(settle_error),
      settle_time(settle_time),
      timeout(timeout)
    {};

    PID(double error, double kp, double ki, double kd, double starti) :
      //secondary constructor
      piderror(error),
      kp(kp),
      ki(ki),
      kd(kd),
      starti(starti)
    {};

    double compute(double error) 
    {
      //integral handling
      if (signbit(error) != signbit(previous_error)) {
        integral = 0; //reset if error crosses zero
      } else {
        if (fabs(error) < starti) {
          //robot is close enough for integral to be useful
          integral += error;
        } else {
          //reset if robot is far away from target
          integral = 0; 
        }
      }
      //output and prep for next iteration
      output = kp*error + ki*integral + kd*(error-previous_error); //calculate PID output
      previous_error = error; //update error values
      if (fabs(error) < settle_error) { //robot is close to target
        time_spent_settled += 10;
      } else { //robot is too far away
        time_spent_settled = 0;
      }
      time_spent_running += 10; //update loop timer
      return output;
   }


    bool is_settled() 
    {
      if (time_spent_running > timeout && timeout != 0) {//running longer than timeout
        //timeout = 0 allows you to have an infinite timeout
        return true; //exit loop
      }
      if (time_spent_settled > settle_time) {//robot is settled at target
        return true; //exit loop
      }
      //continue loop
      return false;
    }

};


void Drive(double right_voltage, double left_voltage) 
{
  DriveR.spin(forward, right_voltage, volt);
  DriveL.spin(forward, left_voltage, volt);
}

void stopDrive(brakeType type = brake) 
{
  DriveL.stop(type);
  DriveR.stop(type);
}

double wrap_angle_180(double deg) 
{
  if (deg > 180) {
    deg -= 360;
  } else if (deg < -180){
    deg += 360;
  }
  return deg;
}


void turn_to_heading(double angle, double max_voltage = 12, double settle_error = 2, double settle_time = 200, double kp = turnkp, double ki = turnki, double kd = turnkd, double starti = 15, double timeout = 4000) 
{
  //Initialize PID class instance
  PID turnPID{ PID(angle, kp, ki, kd, starti, settle_error, settle_time, timeout) };
  while (!turnPID.is_settled()) { //loop while the PID is not settled
    double error{ wrap_angle_180(angle - Inert.heading(degrees)) }; //update error
    double output{ turnPID.compute(error) }; //calculate output to motors

    //ensure that the output does not exceed the motors' maximum voltage
    if (output > max_voltage) {
      output = max_voltage;
    } else if (output < -max_voltage) {
      output = -max_voltage;
    }

    //spin motors at calculated output
    Drive(-output, output);
    wait(10, msec);
  }
  //stop drivetrain when PID is settled
  stopDrive(hold);
}

void swing_left_to_heading(double angle, double max_voltage = 12, double settle_error = 1, double settle_time = 200, double kp = swingkp, double ki = swingki, double kd = swingkd, double starti = 15, double timeout = 4000)
{
  //Initialize PID class instance
  PID swingPID{ PID(angle, kp, ki, kd, starti, settle_error, settle_time, timeout) };
  while (!swingPID.is_settled()) {//loop while the PID is not settled
    double error{ wrap_angle_180(angle - Inert.heading(degrees)) }; //update error
    double output{ swingPID.compute(error) }; //calculate output to motors

    //ensure that the output does not exceed the motors' maximum voltage
    if (output > max_voltage) {
      output = max_voltage;
    } else if (output < -max_voltage) {
      output = -max_voltage;
    }

    //spin left drive motors at calculated output
    Drive(0, output);
    wait(10, msec);
  }
  //stop drivetrain when PID is settled
  stopDrive(hold);
}

void swing_right_to_heading(double angle, double max_voltage = 12, double settle_error = 1, double settle_time = 200, double kp = swingkp, double ki = swingki, double kd = swingkd, double starti = 15, double timeout = 4000) 
{
  //Initialize PID class instance
  PID swingPID{ PID(angle, kp, ki, kd, starti, settle_error, settle_time, timeout) };
  while (!swingPID.is_settled()) {//loop while the PID is not settled
    double error{ wrap_angle_180(angle - Inert.heading(degrees)) };//update error
    double output{ swingPID.compute(error) };//calculate output to motors

    //ensure that the output does not exceed the motors' maximum voltage
    if (output > max_voltage) {
      output = max_voltage;
    } else if (output < -max_voltage) {
      output = -max_voltage;
    }

    //spin right drive motors at calculated output
    Drive(-output, 0);
    wait(10, msec);
  }
  //stop drivetrain when PID is settled
  stopDrive(hold);
}

//converts the integrated motor encoder's degree measurements to inches driven by the robot
double motor_degrees_to_inches(double motor_degrees) {
  return (driving_gear*M_PI*wheel_diameter*motor_degrees)/(driven_gear*360);
}

void drive_forward(double distance, double max_voltage = 12, double settle_error = 1, double settle_time = 200, double kp = drivekp, double ki = driveki, double kd = drivekd, double starti = 8, double timeout = 4000) 
{
  //reset drivetrain encoders
  DriveL.resetPosition();
  DriveR.resetPosition();

  //initialize PID variables
  PID drivePID{ PID(distance, kp, ki, kd, starti, settle_error, settle_time, timeout) };
  double error{ 0 };
  while(!drivePID.is_settled()) { //loop while PID is not settled
    //update error
    error = distance - motor_degrees_to_inches((DriveL.position(degrees)+DriveR.position(degrees))/2);
    double output{ drivePID.compute(error) }; //calculate motor output
    //ensure that the output does not exceed the motors' maximum voltage
    if (output > max_voltage) {
      output = max_voltage;
    } else if (output < -max_voltage) {
      output = -max_voltage;
    }
    //spin motors at calculated output
    Drive(output, output);
    wait(10, msec);
  }
  stopDrive(hold); //stop drivetrain when PID is settled
}

void drive_reverse(double distance, double max_voltage = 12, double settle_error = 1, double settle_time = 200, double kp = drivekp, double ki = driveki, double kd = drivekd, double starti = 8, double timeout = 4000) 
{
  //reset drivetrain encoders
  DriveL.resetPosition();
  DriveR.resetPosition();

  //initialize PID variables
  PID drivePID{ PID(distance, kp, ki, kd, starti, settle_error, settle_time, timeout) };
  double error{ 0 };

  while(!drivePID.is_settled()) {//loop while PID is not settled
    //update error
    error = distance - motor_degrees_to_inches((fabs(DriveL.position(degrees))+fabs(DriveR.position(degrees)))/2);
    double output{ drivePID.compute(error) }; //calculate motor output
    //ensure that the output does not exceed the motors' maximum voltage
    if (output > max_voltage) {
      output = max_voltage;
    } else if (output < -max_voltage) {
      output = -max_voltage;
    }
    //spin motors at calculated output
    Drive(-output, -output);
    wait(10, msec);
  }
  stopDrive(hold); //stop drivetrain when PID is settled
}

void mogo() {
  Mogo.set(!Mogo.value());//set mobile goal holder to opposite state
}

void doink() {
  Doinker.set(!Doinker.value());
}

void reset_mary() {
  MaryLeft.spin(reverse, 5.5, volt);
  MaryRight.spin(reverse, 5.5, volt);
  wait(MaryResetTime, msec);
  MaryLeft.stop();
  MaryRight.stop();
}

void prime_mary() {
  MaryLeft.spin(forward, 5.5, volt);
  MaryRight.spin(forward, 5.5, volt);
  wait(MaryPrimeTime, msec);
  MaryLeft.stop();
  MaryRight.stop();
}

void score_mary() {
  MaryLeft.spin(forward, 5.5, volt);
  MaryRight.spin(forward, 5.5, volt);
  wait(MaryScoreTime, msec);
  MaryLeft.stop();
  MaryRight.stop();
}

void cycle_mary() {
  MaryState += 1;
  MaryState %= 3;
  if (MaryState == 0) {
    reset_mary();
  } else if (MaryState == 1) {
    prime_mary();
  } else {
    score_mary();
  }
}

/*
void skills_auto_mid() {
  drive_reverse(4);
  Mogo.set(true);
  wait(1, sec);
  turn_to_heading(-120);
  Intake.spin(forward, 12, volt);
  drive_forward(24);
  turn_to_heading(-210);
  drive_forward(12);
  wait(1, sec);
  turn_to_heading(-120);
  drive_forward(12);
  wait(1, sec);
  turn_to_heading(-255);
  drive_reverse(36);
  Mogo.set(false);
  drive_forward(12);
}
*/

void match_auto() {
  drive_reverse(34);
  wait(0.2, sec);
  Mogo.set(true);
  turn_to_heading(-60);
  wait(0.5, sec);
  Intake.spin(forward, 12, volt);
  drive_forward(36);
  wait(2, sec);
  wait(1, sec);
  Intake.stop();
}

void intake_spin_forward() {
  Intake.spin(forward);
}

void intake_spin_reverse() {
  Intake.spin(reverse);
}

void intake_control() {
  if (Controller1.ButtonR1.pressing()) {
    //Reverse intake if L1 is pressed
    Intake.spin(Controller1.ButtonL1.pressing() ? reverse : forward, 12, volt);
  } else {
    Intake.stop();
  }
}

void auton() {
  match_auto();
}

void usercontrol() {
  DriveL.setStopping(brake);
  DriveR.setStopping(brake);
  while (1) {
    float throttle = deadband(Controller1.Axis3.position(), 5);
    float turn = deadband(Controller1.Axis1.position(), 5);
    DriveL.spin(forward, (throttle+turn)*.12, volt);
    DriveR.spin(forward, (throttle-turn)*.12, volt);
    wait(20, msec);
    if (!redirect) { //if scoring on mobile goals
      if (Controller1.ButtonR1.pressing()) {
        //spin intake, using the L1 button to control the direction
        Intake.spin(forward, 12, volt);
      } else if (Controller1.ButtonL1.pressing()) {
        Intake.spin(reverse, 12, volt);
      } else {
        Intake.stop();
      }
    }
    

  }
}

int main() {
  // Begin project code
  competition Competition;
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

  MaryLeft.setStopping(brake);
  MaryRight.setStopping(brake);

  Inert.calibrate();
  while (Inert.isCalibrating()) {
    wait(10, msec);
  }

  Controller1.ButtonX.pressed(cycle_mary);
  Controller1.ButtonB.pressed(mogo); //setup mobile goal callback
  Controller1.ButtonDown.pressed(doink);

  while (1) {
    wait(100, msec);
  }
  
}
