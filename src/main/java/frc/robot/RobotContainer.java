// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.IntakeMiddle;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntakeSpin;
import frc.robot.commands.Shooter.ShootFuel;
import frc.robot.commands.autos.AutoIntakeGround;
import frc.robot.commands.autos.AutoIntakeMiddle;
import frc.robot.commands.autos.AutoIntakeSpin;
import frc.robot.commands.autos.AutoShootFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double trimConstant = 1;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Vision vision = new Vision(
        (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)
    );

    // subsystems
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        indexer = new IndexerSubsystem();
        shooter = new ShooterSubsystem();
        intake = new IntakeSubsystem();
        
        autoChooser = configureAutoBuilder();
        configureBindings();
    }

    private SendableChooser<Command> configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> {
                    var pose = drivetrain.getState().Pose;
                    if (pose == null) {
                        return new Pose2d();
                    } 

                    return pose;
                },
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(1, 0, 0), // translation PID — tune these
                    new PIDConstants(1, 0, 0)   // rotation PID — tune these
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
            
            // NamedCommands Initialisation
            NamedCommands.registerCommand("AutoIntakeGround", new AutoIntakeGround(intake));
            NamedCommands.registerCommand("AutoIntakeSpin", new AutoIntakeSpin(intake));
            NamedCommands.registerCommand("AutoIntake", new AutoIntakeGround(intake));
            NamedCommands.registerCommand("AutoShootFuel", new AutoShootFuel(drivetrain, indexer, shooter));
            NamedCommands.registerCommand("AutoIntakeMiddle", new AutoIntakeMiddle(intake));

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
        // left bumper = slow mode, right bumper = snap to nearest bridge position
        drivetrain.setDefaultCommand(
            DriveCommand.joystickDrive(
                drivetrain,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> driver.getRightX(),
                driver.rightTrigger(),
                driver.rightBumper()
            )
        );

        // Intake Bindings
        driver.a().onTrue(new IntakeDown(intake));
        driver.leftTrigger().onTrue(new RunIntake(intake)).onFalse(new StopIntakeSpin(intake));

        // Controller 2
        operator.rightTrigger().whileTrue(new ShootFuel(drivetrain, indexer, shooter));
        operator.leftTrigger().whileTrue(new InstantCommand(() -> {
            indexer.runDutyCycle(-0.5);
            shooter.runKickerDutyCycle(-0.5);
        })).onFalse(new InstantCommand(() -> {
            indexer.runDutyCycle(0);
            shooter.runKickerDutyCycle(0);
        }));
        operator.y().onTrue(new IntakeUp(intake));
        operator.a().onTrue(new IntakeMiddle(intake));
        driver.x().onTrue(Commands.run(() -> {trimConstant = 0.33;}))
        .onFalse(Commands.run(() -> {trimConstant = 1;}));

        driver.b().onTrue(Commands.run(() -> {trimConstant = 2;}))
        .onFalse(Commands.run(() -> {trimConstant = 1;}));

        operator.povRight().whileTrue(Commands.run(() -> {
            shooter.incrementOffset(0.60 * trimConstant);
        }));
        operator.povLeft().whileTrue(Commands.run(() -> {
            shooter.incrementOffset(-0.60 * trimConstant);
        }));

        operator.povUp().whileTrue(Commands.run(() -> {
            intake.incrementOffset(0.5);
        }));
        operator.povDown().whileTrue(Commands.run(() -> {
            intake.incrementOffset(-0.5);
        }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}