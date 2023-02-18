// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include<list>
#include<queue>

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/joystick.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/Victor.h>
#include <frc/BuiltInAccelerometer.h>
#include "rev/CANSparkMax.h"
#include "cameraserver/CameraServer.h"

using namespace std;

// Settings for drive train. Speeed, ID, rotation speed and joystick control.
int leftDriveTrainID = 20, rightDriveTrainID = 21;
int placeholderA = 22, placeholderB = 23, placeholderC = 24, placeholderD = 25, placeholderE = 26;
int motorFChannel = 1;

double driveSpeed = 0.8;
double rotateSpeed = 0.5;
double armSpeed = 0; // currently not in use

bool isJoystick = false;

int avgLen = 4;

list<double> lt(avgLen, 0.0);
queue<double, list<double> > q(lt);

frc::BuiltInAccelerometer rioAccelerometer(frc::Accelerometer::Range::kRange_2G);

frc::XboxController controller(0);
// frc::Joystick joystick(1);
rev::CANSparkMax m_leftDriveTrain{leftDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_rightDriveTrain{rightDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};

// placeholders:
/*
rev::CANSparkMax motorA{placeholderA, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax motorB{placeholderB, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorC{placeholderC, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorD{placeholderD, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorE{placeholderE, rev::CANSparkMax::MotorType::kBrushless};
*/

frc::Victor motorF(1);

frc::DifferentialDrive m_driveTrain{m_leftDriveTrain, m_rightDriveTrain};

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur",640,480);
  
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_leftDriveTrain.RestoreFactoryDefaults();
  m_rightDriveTrain.RestoreFactoryDefaults();

  m_rightDriveTrain.SetInverted(true);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Defult Auto
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto
    m_leftDriveTrain.Set(0.2);
    m_rightDriveTrain.Set(0.2);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  /*
  if (joystick.GetRawAxis(1) < 0) {
    rotateSpeed *= -1;
  }

  if (isJoystick) {
    // Enable joystick control for drive train
    m_driveTrain.ArcadeDrive(joystick.GetRawAxis(1)*driveSpeed, joystick.GetRawAxis(0)*rotateSpeed);
  } else {
    // If not joystick, enable controller control for drive train
  }
  */

  m_driveTrain.ArcadeDrive(controller.GetRawAxis(1)*driveSpeed, controller.GetRawAxis(0)*rotateSpeed);

  double val_y = rioAccelerometer.GetY()*1000 + 10;
  q.push(val_y);
  q.pop();
  // get average of the queue
  double sum = 0.0;
  for (int i = 0; i < avgLen; i++) {
    double elem = q.front();
    sum += elem;
    q.pop(); 
    q.push(elem);
  }
  sum = sum/q.size();

  printf("y = %f, %x\n", sum, q.size());
  
  // if (controller.GetAButtonPressed()) {
    // Toggle joystick control
    // isJoystick = !isJoystick;
    // frc::SmartDashboard::PutString("Test","Test"); 
  // }

  // Motor B and C control up and down of arm (will be finalized later)
  // motorB.Set(controller.GetRawAxis(3)*armSpeed);
  // motorB.Set(controller.GetRawAxis(3)*armSpeed);

  // Motor A controls claw
  // motorA.Set(controller.GetRawAxis(2));

}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {
  // Test code for putting variables onto SmartDashboard
  
  // "Control Mode" displays current controller (true --> joystick, false --> Logitech controller)
  frc::SmartDashboard::PutBoolean("Control Mode", isJoystick);

  // "Drive Speed" displays current speed between 0-1, corresponding to 0%-100% power
  frc::SmartDashboard::PutNumber("Drive Speed", driveSpeed);
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif