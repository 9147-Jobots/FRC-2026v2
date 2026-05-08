// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Drive.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class RobotContainer {

  private final Drive drive;

  private final CommandXboxController controller = new CommandXboxController(0);


  public RobotContainer() {
    drive = new Drive();
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive,
      () -> -controller.getLeftY() * DriveConstants.X_IN * DriveConstants.MAX_LINEAR_SPEED,
      () -> -controller.getLeftX() * DriveConstants.Y_IN * DriveConstants.MAX_LINEAR_SPEED,
      () -> controller.getRightX() * DriveConstants.OMEGA_IN * DriveConstants.MAX_ANGULAR_SPEED));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
