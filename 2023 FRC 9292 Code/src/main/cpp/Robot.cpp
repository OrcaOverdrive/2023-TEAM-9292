// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/joystick.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include "cameraserver/CameraServer.h"

// Settings for drive train. Speeed, ID, rotation speed and joystick control.
int leftDriveTrainID = 20, rightDriveTrainID = 21;
double driveSpeed = 0.8;
double rotateSpeed = 0.5;
bool isJoystick = true;

frc::XboxController controller(0);
frc::Joystick joystick(1);
rev::CANSparkMax m_leftDriveTrain{leftDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_rightDriveTrain{rightDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};

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
    m_leftDriveTrain.Set(0.1);
    m_rightDriveTrain.Set(0.1);
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  if (joystick.GetRawAxis(1) < 0){
    rotateSpeed *= -1;
  }

  if (isJoystick) {
    // Enable joystick control for drive train
    m_driveTrain.ArcadeDrive(joystick.GetRawAxis(1)*driveSpeed, joystick.GetRawAxis(0)*rotateSpeed);
  } else {
    // If not joystick, enable controller control for drive train
    m_driveTrain.ArcadeDrive(controller.GetRawAxis(1)*driveSpeed, controller.GetRawAxis(0)*rotateSpeed);
  }

  if (controller.GetAButtonPressed()) {
    // Toggle joystick control
    isJoystick = !isJoystick;
    frc::SmartDashboard::PutString("Test","Test"); 
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif