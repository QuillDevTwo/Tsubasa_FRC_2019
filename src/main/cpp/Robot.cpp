/* 
TODO:
Slow down prints so it's easier to see what's going on.

*/

//Custom Headers
#include "Robot.h"

//Input/Output Stream
#include <iostream>

//Dashboard
#include <frc/smartdashboard/SmartDashboard.h>

//Parts of the Robot
#include <thread>
#include <cameraserver/CameraServer.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <wpi/raw_ostream.h>
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>

#if defined(__linux__)
  static void VisionThread() {
    // Get the USB camera from CameraServer
    cs::UsbCamera camera =
        frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(320, 240);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Align", 320, 240);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it
      // in the source mat.  If there is an error notify the
      // output.
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      // Put a rectangle on the image
       rectangle(mat, cv::Point(150, 0), cv::Point(170, 240),
                cv::Scalar(0, 255, 255), 5);
      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
  }
#endif
//Create Pointers...

//Motors
frc::PWMVictorSPX *leftDrive;
frc::PWMVictorSPX *rightDrive;

//Joystick
frc::Joystick *stick;

//Pneumatics
//Compressor
frc::Compressor *air;

//Solenoid
frc::Solenoid *cargoShot;
frc::DoubleSolenoid *rocketShot;

//Init Robot
void Robot::RobotInit() {

#if defined(__linux__)
    std::thread visionThread(VisionThread);
    visionThread.detach();
#else
    wpi::errs() << "Vision only available on Linux.\n";
    wpi::errs().flush();
#endif

  //Declare pointer types...
  stick = new frc::Joystick(0);
  
  //Drive Train Sides
  leftDrive = new frc::PWMVictorSPX(0);
  rightDrive = new frc::PWMVictorSPX(1);

  //Air Compressor
  air = new frc::Compressor(0);

  //Solenoids
  cargoShot = new frc::Solenoid(0);
  rocketShot = new frc::DoubleSolenoid(1, 2);

  air->Start();
}


void driveStraightForward(){
  leftDrive->Set(.5); stick->SetRumble(frc::GenericHID::kLeftRumble, .5);
  rightDrive->Set(.5); stick->SetRumble(frc::GenericHID::kRightRumble, .5);
}

void driveStraightBackward(){
  leftDrive->Set(-.5); stick->SetRumble(frc::GenericHID::kLeftRumble, .5);
  rightDrive->Set(-.5); stick->SetRumble(frc::GenericHID::kRightRumble, .5);
}

void driveLeftOnly(){
  leftDrive->Set(.5);
}

void driveRightOnly() {
  rightDrive->Set(.5);
}

//Throw Drive Controls in here later?
void DriveControls() {

}
void printComp() {
 bool compressorOn = air->Enabled();
 bool pressureSwitch = air->GetPressureSwitchValue();
 double current = air->GetCompressorCurrent();
 std::cout << compressorOn << std::endl;
 std::cout << pressureSwitch << std::endl;
 std::cout << current << std::endl;
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  //Manage Sticks
  float leftY = stick->GetRawAxis(1);
  float rightY = stick->GetRawAxis(5);

  //Create Controller Buttons
  bool aButton = stick->GetRawButton(1); //A
  bool bButton = stick->GetRawButton(2); //B
  bool xButton = stick->GetRawButton(3); //X
  bool yButton = stick->GetRawButton(4); //Y

  //Bumpers
  bool lBumper = stick->GetRawButton(5); //Left Bumper
  bool rBumper = stick->GetRawButton(6); //Right Bumper

  //Start and Select
  bool select = stick->GetRawButton(7); //Select
  bool start = stick->GetRawButton(8); //Start

  //Set POVs
  int povUP = 0; //UP
  int povRIGHT = 90; //DOWN
  int povDOWN = 180; //LEFT
  int povLEFT = 270; //RIGHT



  //Manage the speed of the Left Drive train.
  const float dz = .2;
  if(std::abs(leftY) > dz){
    leftDrive->Set(-leftY*.7);
    std::cout << "L. Speed: " << leftDrive->Get() << std:: endl;
    stick->SetRumble(frc::GenericHID::kLeftRumble, std::abs(leftY));
  } else if(leftY == 1){
    leftDrive->Set(-leftY);
  } 
  else {leftDrive->Set(0); stick->SetRumble(frc::GenericHID::kLeftRumble, 0);}

  if(std::abs(rightY) > dz){
    rightDrive->Set(-rightY*.6);
    std::cout << "R. Speed: " << rightDrive->Get() << std:: endl;
    stick->SetRumble(frc::GenericHID::kRightRumble, std::abs(rightY));
  }else if(rightY == 1){
    rightDrive->Set(rightY);
  } 
  else {rightDrive->Set(0);stick->SetRumble(frc::GenericHID::kRightRumble, 0);}

  //Percise DPAD Movements.
  if(stick->GetPOV() == povUP){driveStraightForward();}  // Forward
  if(stick->GetPOV() == povDOWN){driveStraightBackward();} // Backward
  if(stick->GetPOV() == povLEFT){driveRightOnly();} // Right
  if(stick->GetPOV() == povRIGHT){driveLeftOnly();} // Left

  //Solenoid Control

  //Control Cargo Shot Solenoid (1x Solenoid) - PCM SLOT 1
  if(aButton){
    cargoShot->Set(true);
  } else {
    cargoShot->Set(false);
  }

  //Control Rocket Shooter Solenoid (2x Solenoid) - PCM SLOT 2/3
  if(rBumper) {
     rocketShot->Set(frc::DoubleSolenoid::kForward);
  } else if (lBumper) {
    rocketShot->Set(frc::DoubleSolenoid::kReverse);
  } else {
    rocketShot->Set(frc::DoubleSolenoid::kOff);
  }

  //HARD RESET BLOCK
  if(start) {
    leftDrive->Set(0);
    rightDrive->Set(0);
    rocketShot->Set(frc::DoubleSolenoid::kOff);
    cargoShot->Set(false);
    stick->SetRumble(frc::GenericHID::kRightRumble, 0);
    stick->SetRumble(frc::GenericHID::kLeftRumble, 0);
  }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
