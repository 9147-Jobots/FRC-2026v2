// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand {
  private DriveCommand() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      CommandSwerveDrivetrain drivetrain,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier slowMode,
      BooleanSupplier snapToBridge) {
    double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    ProfiledPIDController snapYController = new ProfiledPIDController(
        DriveCommandConstants.SNAP_Y_KP, 0, 0,
        new TrapezoidProfile.Constraints(
            DriveCommandConstants.SNAP_Y_MAX_VELOCITY,
            DriveCommandConstants.SNAP_Y_MAX_ACCELERATION));

    SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * DriveCommandConstants.DEADBAND)
        .withRotationalDeadband(MaxAngularRate * DriveCommandConstants.DEADBAND)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    return drivetrain.applyRequest(() -> {
      double speedMult = slowMode.getAsBoolean() ? DriveCommandConstants.SLOW_SPEED_MULTIPLIER
          : DriveCommandConstants.DEFAULT_SPEED_MULTIPLIER;
      double turnMult = slowMode.getAsBoolean() ? DriveCommandConstants.SLOW_TURN_MULTIPLIER
          : DriveCommandConstants.DEFAULT_TURN_MULTIPLIER;

      double currentY = drivetrain.getState().Pose.getY();
      double velocityY;
      if (snapToBridge.getAsBoolean()) {
        double dist1 = Math.abs(currentY - DriveCommandConstants.SNAP_Y_BRIDGE_POS1);
        double dist2 = Math.abs(currentY - DriveCommandConstants.SNAP_Y_BRIDGE_POS2);
        double target = dist1 <= dist2 ? DriveCommandConstants.SNAP_Y_BRIDGE_POS1 : DriveCommandConstants.SNAP_Y_BRIDGE_POS2;
        snapYController.setGoal(target);
        velocityY = snapYController.calculate(currentY);
      } else {
        snapYController.reset(currentY);
        velocityY = -ySupplier.getAsDouble() * MaxSpeed * DriveCommandConstants.Y_IN * speedMult;
      }

      return drive
          .withVelocityX(-xSupplier.getAsDouble() * MaxSpeed * DriveCommandConstants.X_IN * speedMult)
          .withVelocityY(velocityY)
          .withRotationalRate(-omegaSupplier.getAsDouble() * MaxAngularRate * DriveCommandConstants.OMEGA_IN * turnMult);
    });
  }
}
