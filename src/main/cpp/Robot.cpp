// Copyright (c) 2023-2024 CurtinFRC
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

#include "Robot.h"

// include units
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <vision/Limelight.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/controller/RamseteController.h>
#include <frc/Timer.h>

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption("Default Auto", "Default Auto");

  frc::SmartDashboard::PutData("Auto Selector", &m_chooser);

  m_path_chooser.SetDefaultOption("Path1", "paths/output/Path1.wpilib.json");

  m_path_chooser.AddOption("Path1", "paths/output/Path1.wpilib.json");
  m_path_chooser.AddOption("Path2", "paths/output/Path2.wpilib.json");

  frc::SmartDashboard::PutData("Path Selector", &m_path_chooser);

  frc::SmartDashboard::PutData("Field", &m_field);

  simulation_timer = frc::Timer();

  robotmap.swerveBase.gyro->Reset();

  limelight = new wom::Limelight("imblindedbythelights");
  limelight->SetPipeline(wom::LimelightPipeline::kPipeline0);
  limelight->SetCamMode(wom::LimelightCamMode::kVisionProcessor);
  limelight->SetLEDMode(wom::LimelightLEDMode::kForceOn);
  currentPosition = limelight->GetPose();
  
  
  

  _swerveDrive =
      new wom::SwerveDrive(robotmap.swerveBase.config, frc::Pose2d());
  wom::BehaviourScheduler::GetInstance()->Register(_swerveDrive);
  _swerveDrive->SetDefaultBehaviour([this]() {
    return wom::make<wom::ManualDrivebase>(_swerveDrive,
                                           &robotmap.controllers.driver);
  });

  // m_driveSim = new wom::TempSimSwerveDrive(&simulation_timer, &m_field);
  // m_driveSim = wom::TempSimSwerveDrive();
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  getv = limelight->GetTargetingData(wom::LimelightTargetingData::kTv);
  getx = limelight->GetTargetingData(wom::LimelightTargetingData::kTx);
  gety = limelight->GetTargetingData(wom::LimelightTargetingData::kTy);

  loop.Poll();
  wom::BehaviourScheduler::GetInstance()->Tick();

  _swerveDrive->OnUpdate(dt);

  currentPosition = limelight->GetPose();
  inviewapriltag = limelight->GetAprilTagData(wom::LimelightAprilTagData::kTid)[0];
}

void Robot::AutonomousInit() {
  // m_driveSim->SetPath(m_path_chooser.GetSelected());

  loop.Clear();
  sched->InterruptAll();
  // _swerveDrive->OnStart();
}

void Robot::AutonomousPeriodic() {
  // m_driveSim->OnUpdate();

  _swerveDrive->SetPose(frc::Pose2d(units::meter_t(getx) + currentPosition.X(), currentPosition.Y(), 1_rad));
}

void Robot::TeleopInit() {
  // _swerveDrive->OnStart();
  // sched->InterruptAll();
}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}
