// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.jni.UtilsJNI;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.services.ShooterService;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  

  private final RobotContainer m_robotContainer;
  private Field2d field = new Field2d();

  public Robot() {
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Is In Middle", ShooterService.isInMiddle(m_robotContainer.drivetrain));
    SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

    Optional<Pose2d> optionalRobotPose = m_robotContainer.drivetrain.samplePoseAt(UtilsJNI.getCurrentTimeSeconds());

    Pose2d robotPose;
    if (optionalRobotPose.isPresent()) {
        robotPose = optionalRobotPose.get();
    } else {
        robotPose = new Pose2d();
    }
    field.setRobotPose(robotPose);

    String m_lastAuto = null;
    try {
      String selectedAuto = m_robotContainer.autoChooser.getSelected().getName();

      if (!DriverStation.isEnabled()) {
        if (!selectedAuto.equals(m_lastAuto)) {
          m_lastAuto = selectedAuto;

          List<Pose2d> autoPoses = new ArrayList<>();

          try {
            List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto);
                        
            if (paths != null) {
              for (PathPlannerPath path : paths) {
                autoPoses.addAll(path.getPathPoses());
              }
            }
                        
            } catch (Exception e) {
              DriverStation.reportError("Unexpected error loading auto: " + selectedAuto, e.getStackTrace());
            }
                            
            // Plot onto field
            field.getObject("Autonomous Path").setPoses(autoPoses);
        }
      }
    } catch (Exception e) {}
        
    SmartDashboard.putData("Field", field);

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
