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
#include <frc/motorcontrol/Spark.h>
#include "rev/CANSparkMax.h"
#include "cameraserver/CameraServer.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include "frc/Compressor.h"
#include "frc/DoubleSolenoid.h"

#include <frc/motorcontrol/Spark.h>

using namespace std;

// ID for motors
int leftLeadMotorID = 60, rightLeadMotorID = 61; // I don't know why they needed to be changed, but it works
int leftFollowMotorID = 24, rightFollowMotorID = 25;
int armLeadID = 22, armFollowID = 23;

// PWM port for linear actuator
int clawChannel = 0;

// int placeholderC = 24, placeholderD = 25, placeholderE = 26;

double driveSpeed = 0;
double turnSpeed = 0;
double armSpeed = 0;

double fastSetPoint = 0.8;
double perciseSetPoint = 0.55;
double armSetPoint = 0.25;
double rotateSpeed = 0.5;

double increment = 0.05;

double armIncrement = 0.015;

double movePercent = 0.2;
double turnPercent = 0.05;
double armPercent = 0.2;

double autoSpeed; // hard-coded speed
double maxAutoSpeed = 0.5; // not in use

bool isJoystick = false;
bool autoOnRamp = false;
bool controlsReverse = false;
bool persisionMode = false;

int avgLen = 4;

list<int> lt(avgLen, 0);
queue<int, list<int> > q(lt);

frc::Timer *timer = new frc::Timer();

frc::BuiltInAccelerometer rioAccelerometer(frc::Accelerometer::Range::kRange_2G);

frc::XboxController controller(0);
// frc::Joystick joystick(1);

rev::CANSparkMax m_leftLeadMotor{leftLeadMotorID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_rightLeadMotor{rightLeadMotorID, rev::CANSparkMax::MotorType::kBrushed};

rev::CANSparkMax m_leftFollowMotor{leftFollowMotorID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_rightFollowMotor{rightFollowMotorID, rev::CANSparkMax::MotorType::kBrushed};

rev::CANSparkMax m_armLeadMotor{armLeadID, rev::CANSparkMax::MotorType::kBrushed};
rev::CANSparkMax m_armFollowMotor{armFollowID, rev::CANSparkMax::MotorType::kBrushed};

frc::Spark m_claw{clawChannel};

frc::DoubleSolenoid pcmSolenoid{10, frc::PneumaticsModuleType::REVPH, 13, 15};

frc::Compressor pcmCompresser{10, frc::PneumaticsModuleType::REVPH};

// placeholders:
/*
rev::CANSparkMax motorC{placeholderC, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorD{placeholderD, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax motorE{placeholderE, rev::CANSparkMax::MotorType::kBrushless};
*/

frc::DifferentialDrive m_driveTrain{m_leftLeadMotor, m_rightLeadMotor};

// Shuffleboard testing code
frc::ShuffleboardTab& testingTab = frc::Shuffleboard::GetTab("Testing");
// testingTab.Add("Random Axis", controller.GetRawAxis(0));

frc::SimpleWidget& testingWidget = testingTab.Add("Random Axis", controller.GetRawAxis(0));

void toggleBool(bool *boolVar, bool buttonPressed) {
  if (buttonPressed) {
    *boolVar = !(*boolVar);   
  }
}

double incrementAmount(double diff, double i, double percent) {
  if (diff > 0) {
    return percent*diff + i;
  } else if (diff < 0) {
    return percent*diff - i;
  }

  // alternetive code??
  // return iPercent*diff + (diff/abs(diff))*i;
}

signed int getBuiltInAccelerometer() {
  int val_x = (int) (rioAccelerometer.GetZ()*1000 - 15);
  // printf("val_x: %d\n", val_x);
  // printf("prev = %i, cur = %i\n", q.back(), val_x);
  if (val_x - q.back() >= 50) {
    q.push(q.back()+50);
    q.pop();
  } else if (val_x - q.back() <= -50) {
    q.push(q.back()-50);
    q.pop();
  } else {
    q.push(val_x);
    q.pop();
  }

  // get average of the queue
  int sum = 0;
  for (int i = 0; i < avgLen; i++) {
    int elem = q.front();
    sum += elem;
    q.pop(); 
    q.push(elem);
  }
  int average = (int) (sum/avgLen);
  return average;
}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption("Default", kAutoNameDefault);
  m_chooser.AddOption("Periodic Accelerometer", "Periodic Accelerometer");
  m_chooser.AddOption("Push Cube", "Push Cube");
  m_chooser.AddOption("Charge Station", "Charge Station");

  frc::CameraServer::StartAutomaticCapture();
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Blur",640,480);
  
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_leftLeadMotor.RestoreFactoryDefaults();
  m_leftFollowMotor.RestoreFactoryDefaults();
  m_rightLeadMotor.RestoreFactoryDefaults();
  m_rightFollowMotor.RestoreFactoryDefaults();

  m_rightLeadMotor.SetInverted(true);
  m_rightFollowMotor.SetInverted(true);

  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);
  m_armFollowMotor.Follow(m_armLeadMotor);

  // pnematics
  pcmCompresser.EnableDigital();
  pcmSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
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
    timer->Start();
    // printf("starting\n");
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == "Periodic Accelerometer") {
    signed int angle = getBuiltInAccelerometer();
    // printf("x, onRamp: %i, %s\n", angle, autoOnRamp ? "true" : "false");
    if (angle < -300) {
      autoOnRamp = true;
    }
    if (!autoOnRamp || angle <= -50) {
      m_driveTrain.TankDrive(-0.5, -0.5);
    } else if (autoOnRamp && angle > -50) {
      m_driveTrain.TankDrive(0, 0);
    }
  } else if (m_autoSelected == "Push Cube") {
    float time = timer->Get().value();

    if (time <= 0.8) {
      autoSpeed = 0;
      m_armLeadMotor.Set(-0.15);

    } else if (time <= 6) {
      m_armLeadMotor.Set(0);
      autoSpeed = 0.5;
    } else {
      autoSpeed = 0;
    }

    m_driveTrain.TankDrive(autoSpeed, autoSpeed);

  } else if (m_autoSelected == "Charge Station") {
    // Default Auto
    // units::time::second_t curTime = timer->Get();
    float time = timer->Get().value();
    // printf("current time: %f\n", time);

    if (time <= 0.8) {
      autoSpeed = 0;
      m_armLeadMotor.Set(-0.15);
    if (time <= 3) {
      autoSpeed = 0.8;
      m_armLeadMotor.Set(0);
    } else if (time <= 5.25) {
      autoSpeed = 0.45;
    // } else if (time <= 6) {
    //   autoSpeed = 0;
    // } else if(time <= 9.5) {
    //   autoSpeed = -0.7;
    } else {
      autoSpeed = 0;
    }

    // printf("hello\n");

    m_driveTrain.TankDrive(autoSpeed, autoSpeed);
  } else {
    float time = timer->Get().value();

    if (time <= 6) {
      m_driveTrain.TankDrive(0.5, 0.5);
    } else {
      m_driveTrain.TankDrive(0, 0);
    }
  }
  } 
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  // signed int ac_val = (signed int) (rioAccelerometer.GetZ()*1000-15);
  // printf("val_x: %i\n", ac_val);

  toggleBool(&controlsReverse, controller.GetRawButtonPressed(1));
  toggleBool(&persisionMode, controller.GetRawButtonPressed(2));

  double targetForwardSpeed = controller.GetRawAxis(1)* ((persisionMode) ? perciseSetPoint : fastSetPoint);
  double targetTurnSpeed = controller.GetRawAxis(0)* ((persisionMode) ? perciseSetPoint : fastSetPoint);
  double targetArmPosition = controller.GetRawAxis(3)*armSetPoint;

  // adjust drive speed
  // if (targetForwardSpeed > driveSpeed) {
  //   // speed must increase to get closer towards target
  //   driveSpeed += increment;
  // } else if (targetForwardSpeed < driveSpeed) {
  //   // speed must decrease to get closer towards target
  //   driveSpeed -= increment;
  // }

  driveSpeed += incrementAmount(targetForwardSpeed - driveSpeed, increment, movePercent);

  // adjust turn speed
  // if (targetTurnSpeed > turnSpeed) {
  //   // turn speed must increase to get closer towards target
  //   turnSpeed += increment;
  // } else if (targetTurnSpeed < turnSpeed) {
  //   // turn speed must decrease to get closer towards target
  //   turnSpeed -= increment;
  // }

  turnSpeed += incrementAmount(targetTurnSpeed - turnSpeed, increment, turnPercent);

  // adjust armSpeed
  // if (targetArmPosition > armSpeed) {
  //   // turn speed must increase to get closer towards target
  //   armSpeed += armIncrement;
  // } else if (targetArmPosition < armSpeed) {
  //   // turn speed must decrease to get closer towards target
  //   armSpeed -= armIncrement;
  // }

  armSpeed += incrementAmount(targetArmPosition - armSpeed, increment, armPercent);

  // printf("x = %i\n", getBuiltInAccelerometer());
  
  // if (controller.GetAButtonPressed()) {
    // Toggle joystick control
    // isJoystick = !isJoystick;
    // frc::SmartDashboard::PutString("Test","Test"); 
  // }

  // pnematics control
  if (controller.GetRawButtonPressed(4)) {
    pcmSolenoid.Toggle();
  }

  if (controller.GetRawButton(7)) {
    m_claw.Set(1);
  } else if (controller.GetRawButton(8)) {
    m_claw.Set(-1);
  } else {
    m_claw.Set(0);
  }
    
  // m_claw.Set(controller.GetRawAxis(2));
  
  if (controlsReverse) {
    m_armLeadMotor.Set(-armSpeed);
  } else {
    m_armLeadMotor.Set(armSpeed);
  }

  // if (isJoystick) {
  //   // Enable joystick control for drive train
  //   m_driveTrain.ArcadeDrive(joystick.GetRawAxis(1)*driveSpeed, joystick.GetRawAxis(0)*((joystick.GetRawAxis(1) < 0) ? -rotateSpeed : rotateSpeed));
  // } else {
  //   // If not joystick, enable controller control for drive train
  //    // }

  if (controlsReverse) {
    m_driveTrain.ArcadeDrive(-driveSpeed, turnSpeed, false);
  } else {
    m_driveTrain.ArcadeDrive(driveSpeed, turnSpeed, false);
  }

  // Emergency
  // if (controller.GetRawButton(3)) {
  //   m_driveTrain.ArcadeDrive(controller.GetLeftY()*0.7, controller.GetLeftX()*0.7);
  // }

  // put important data onto smartdashboard
  frc::SmartDashboard::PutNumber("targetForwardSpeed", targetForwardSpeed);
  frc::SmartDashboard::PutNumber("driveSpeed", driveSpeed);
  frc::SmartDashboard::PutNumber("targetTurnSpeed", targetTurnSpeed);
  frc::SmartDashboard::PutNumber("turnSpeed", turnSpeed);
  frc::SmartDashboard::PutNumber("Accelerometer", getBuiltInAccelerometer());
  frc::SmartDashboard::PutBoolean("controlsReverse", controlsReverse);
  frc::SmartDashboard::PutBoolean("persisionMode", persisionMode);
  frc::SmartDashboard::PutBoolean("Pnumatics Toggle", controller.GetRawButton(4));

  // put differential drive data (hopefully appears as a widget), not tested
  frc::SmartDashboard::PutData("Differential Drive", &m_driveTrain);
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