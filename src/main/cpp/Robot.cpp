//Custom Headers
#include "Robot.h"

//Input/Output Stream
#include <iostream>

//Dashboard
#include <frc/smartdashboard/SmartDashboard.h>

//Parts of the Robot
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/drive/DifferentialDrive.h>

#if defined(__linux__)

 private:
  static void VisionThread() {
    // Get the USB camera from CameraServer
    cs::UsbCamera camera =
        frc::CameraServer::GetInstance()->StartAutomaticCapture();
    // Set the resolution
    camera.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::GetInstance()->PutVideo("Rectangle", 640, 480);

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
      rectangle(mat, cv::Point(100, 100), cv::Point(400, 400),
                cv::Scalar(255, 255, 255), 5);
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
  stick = new Joystick(0);

  leftDrive = new PWMVictorSPX(1);
  rightDrive = new PWMVictorSPX(2);

  const float dz = .2;
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
  float leftY = stick->GetRawAxis(0);
  float rightY = stick->GetRawAxis(3);

  if(std::abs(leftY) > dz){
    leftDrive->Set(leftY);
    std::cout << "L. Speed: " << leftDrive->Get() << std:: endl;
  } else {leftDrive->Set(0);}

  if(std::abs(rightY) > dz){
    rightDrive->Set(rightY);
    std::cout << "R. Speed: " << rightDrive->Get() << std:: endl;
  } else {rightDrive->Set(0);}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
