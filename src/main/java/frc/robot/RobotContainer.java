// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Intake.IntakeFuel;
import frc.robot.commands.Intake.IntakeRest;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.StopIntakeSpin;
import frc.robot.commands.Shooter.ShootFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.services.ShooterService;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Vision vision = new Vision(
        (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)
    );

    // subsystems
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = configureAutoBuilder();

        indexer = new IndexerSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();

        configureBindings();
    }

    private SendableChooser<Command> configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0), // translation PID — tune these
                    new PIDConstants(7, 0, 0)   // rotation PID — tune these
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed: " + e.getMessage(), e.getStackTrace());
        }

        SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // left bumper = slow mode, right bumper = fast mode
        drivetrain.setDefaultCommand(
            DriveCommand.joystickDrive(
                drivetrain,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> controller.getRightX(),
                controller.leftBumper(),
                controller.rightBumper()
            )
        );

        // Intake Bindings
        controller.rightTrigger().whileTrue(new IntakeFuel(intake)).onFalse(new StopIntakeSpin(intake));
        controller.y().onTrue(new IntakeRest(intake));
        controller.x().onTrue(new IntakeUp(intake));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);

        controller.x().onTrue(Commands.runOnce(() -> ShooterService.runShooterKicker(shooter))).onFalse(Commands.runOnce(() -> ShooterService.stopShooterKicker(shooter)));
        controller.leftTrigger().whileTrue(new ShootFuel(drivetrain, indexer, shooter));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
