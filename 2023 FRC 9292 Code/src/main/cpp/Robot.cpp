// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <list>
#include <queue>

#include "Robot.h"

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/joystick.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/BuiltInAccelerometer.h>
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"
#include "cameraserver/CameraServer.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/shuffleboard/ComplexWidget.h>

using namespace std;

// Settings for drive train. Speeed, ID, rotation speed and joystick control.
int leftDriveTrainID = 20, rightDriveTrainID = 21;
int armID = 23, clawID = 22;
// int placeholderC = 24, placeholderD = 25, placeholderE = 26;

frc::Timer t;

double driveSpeed = 0;
double turnSpeed = 0;
double armSpeed = 0;

double fastSetPoint = 0.6;
double perciseSetPoint = 0.2;
double armSetPoint = 0.17;
double rotateSpeed = 0.5;
double clawSpeed = 1.0; 

double increment = 0.04;
double armIncrement = 0.02;
double maxAutoSpeed = 0.5; // not in use

bool isJoystick = false;
bool autoOnRamp = false;
bool controlsReverse = false;

int avgLen = 4;

list<double> lt(avgLen, 0.0);
queue<double, list<double> > q(lt);

frc::BuiltInAccelerometer rioAccelerometer(frc::Accelerometer::Range::kRange_2G);

frc::XboxController controller(0);
// frc::Joystick joystick(1);
rev::CANSparkMax m_leftDriveTrain{leftDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_rightDriveTrain{rightDriveTrainID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_arm{armID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_claw{clawID, rev::CANSparkMax::MotorType::kBrushed};

// placeholders:
/*
rev::CANSparkMax motorC{placeholderC, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorD{placeholderD, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorE{placeholderE, rev::CANSparkMax::MotorType::kBrushless};
*/

frc::DifferentialDrive m_driveTrain{m_leftDriveTrain, m_rightDriveTrain};

// Shuffleboard testing code
frc::ShuffleboardTab& testingTab = frc::Shuffleboard::GetTab("Testing");
// testingTab.Add("Random Axis", controller.GetRawAxis(0));

frc::SimpleWidget& testingWidget = testingTab.Add("Random Axis", controller.GetRawAxis(0));

int getBuiltInAccelerometer() {
  double val_x = rioAccelerometer.GetX()*1000 + 20;
  printf("val_x: %i\n", (int) (val_x));
  printf("prev = %d, cur = %d\n", q.back(), val_x);
  if (abs(val_x-q.back()) <= 50) {
    q.push(val_x);
    q.pop();
  }

  // get average of the queue
  double sum = 0.0;
  for (int i = 0; i < avgLen; i++) {
    double elem = q.front();
    sum += elem;
    q.pop(); 
    q.push(elem);
  }
  int average = (int) (sum/avgLen);
  return average;
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption("Default", kAutoNameDefault);
  m_chooser.AddOption("Custom", kAutoNameCustom);
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
  // m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto
  } else {
    // Defult Auto
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    int angle = getBuiltInAccelerometer();
    printf("x, onRamp: %i, %s\n", angle, autoOnRamp ? "true" : "false");
    if (angle < -300) {
      autoOnRamp = true;
    }
    if (!autoOnRamp || angle <= -50) {
      m_driveTrain.TankDrive(-0.5, -0.5);
    } else if (autoOnRamp && angle > -50) {
      m_driveTrain.TankDrive(0, 0);
    }

  } else {
    // Default Auto
    m_driveTrain.TankDrive(0.1, 0.1);
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

  double targetForwardSpeed = controller.GetRawAxis(1)*fastSetPoint;
  double targetTurnSpeed = controller.GetRawAxis(0)*fastSetPoint;
  double targetArmPosition = controller.GetRawAxis(3)*fastSetPoint;

  // adjust drive speed
  if (targetForwardSpeed > driveSpeed) {
    // speed must increase to get closer towards target
    driveSpeed += increment;
  } else if (targetForwardSpeed < driveSpeed) {
    // speed must decrease to get closer towards target
    driveSpeed -= increment;
  }

  // adjust turn speed
  if (targetTurnSpeed > turnSpeed) {
    // turn speed must increase to get closer towards target
    turnSpeed += increment;
  } else if (targetTurnSpeed < turnSpeed) {
    // turn speed must decrease to get closer towards target
    turnSpeed -= increment;
  }

  // adjust armSpeed
  if (targetArmPosition > armSpeed) {
    // turn speed must increase to get closer towards target
    armSpeed += armIncrement;
  } else if (targetArmPosition < armSpeed) {
    // turn speed must decrease to get closer towards target
    armSpeed -= armIncrement;
  }

  // toggle reverse controls depending on button A
  if (controller.GetAButtonPressed()) {
    controlsReverse = !controlsReverse;
  }

  if (controlsReverse) {
    m_driveTrain.ArcadeDrive(driveSpeed, turnSpeed);
  } else {
    m_driveTrain.ArcadeDrive(driveSpeed, turnSpeed);
  }

  // printf("x = %i\n", getBuiltInAccelerometer());
  
  // if (controller.GetAButtonPressed()) {
    // Toggle joystick control
    // isJoystick = !isJoystick;
    // frc::SmartDashboard::PutString("Test","Test"); 
  // }

  // m_arm controls up and down of arm
  m_claw.Set(controller.GetRawAxis(2)*clawSpeed);

  // m_claw controls claw
  m_arm.Set(armSpeed);

  // put important data onto smartdashboard
  frc::SmartDashboard::PutNumber("targetForwardSpeed", targetForwardSpeed);
  frc::SmartDashboard::PutNumber("driveSpeed", driveSpeed);
  frc::SmartDashboard::PutNumber("targetTurnSpeed", targetTurnSpeed);
  frc::SmartDashboard::PutNumber("turnSpeed", driveSpeed);
  frc::SmartDashboard::PutBoolean("controlsReverse", controlsReverse);

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
  
  // "Accelerometer Value" displays current average of most recent accelerometer readings
  frc::SmartDashboard::PutNumber("Accelrometer Value", getBuiltInAccelerometer());

  // frc::SimpleWidget& testingWidget = frc::Shuffleboard::GetTab("Testing").Add("Random Axis", (int) (controller.GetRawAxis(0)*100));

  // frc::Shuffleboard::Update();

  nt::GenericEntry * test = frc::Shuffleboard::GetTab("Testing").Add("Test Slider", 0).WithWidget(frc::BuiltInWidgets::kNumberSlider).GetEntry();
  // nt::Value ntTestVal = nt::GenericEntry::ValueType;
  // printf("Test Slider Value: %d", *test);
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif