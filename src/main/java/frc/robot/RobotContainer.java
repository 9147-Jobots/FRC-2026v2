// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.jni.UtilsJNI;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Intake.IntakeDown;
import frc.robot.commands.Intake.IntakeFuel;
import frc.robot.commands.Intake.IntakeMiddle;
import frc.robot.commands.Intake.IntakeUp;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntakeSpin;
import frc.robot.commands.Shooter.ShootFuel;
import frc.robot.commands.autos.AutoIntake;
import frc.robot.commands.autos.AutoIntakeMiddle;
import frc.robot.commands.autos.AutoShootFuel;
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
    private double trimConstant = 1;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController controller = new CommandXboxController(0);

    private final CommandXboxController controller1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Vision vision = new Vision(
        (pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)
    );

    // subsystems
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private final SendableChooser<Command> autoChooser;

    private Field2d field = new Field2d();

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
            NamedCommands.registerCommand("AutoIntake", new AutoIntake(intake));
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
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> controller.getRightX(),
                controller.rightTrigger(),
                controller.rightBumper()
            )
        );

        // Intake Bindings
        controller.a().onTrue(new IntakeDown(intake));
        controller.leftTrigger().onTrue(new RunIntake(intake)).onFalse(new StopIntakeSpin(intake));

        // Controller 2
        controller1.rightTrigger().whileTrue(new ShootFuel(drivetrain, indexer, shooter));
        controller1.leftTrigger().whileTrue(new InstantCommand(() -> {
            indexer.runDutyCycle(-0.5);
            shooter.runKickerDutyCycle(-0.5);
        })).onFalse(new InstantCommand(() -> {
            indexer.runDutyCycle(0);
            shooter.runKickerDutyCycle(0);
        }));
        controller1.y().onTrue(new IntakeUp(intake));
        controller1.a().onTrue(new IntakeMiddle(intake));
        controller.x().onTrue(Commands.run(() -> {trimConstant = 0.33;}))
        .onFalse(Commands.run(() -> {trimConstant = 1;}));

        controller.b().onTrue(Commands.run(() -> {trimConstant = 2;}))
        .onFalse(Commands.run(() -> {trimConstant = 1;}));

        controller1.povRight().whileTrue(Commands.run(() -> {
            shooter.incrementOffset(0.60 * trimConstant);
        }));
        controller1.povLeft().whileTrue(Commands.run(() -> {
            shooter.incrementOffset(-0.60 * trimConstant);
        }));

        controller1.povUp().whileTrue(Commands.run(() -> {
            intake.incrementOffset(0.5);
        }));
        controller1.povDown().whileTrue(Commands.run(() -> {
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

    public void periodic() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Is In Middle", ShooterService.isInMiddle(drivetrain));
        SmartDashboard.putNumber("Battery", RobotController.getBatteryVoltage());

        Optional<Pose2d> optionalRobotPose = drivetrain.samplePoseAt(UtilsJNI.getCurrentTimeSeconds());

            Pose2d robotPose;
            if (optionalRobotPose.isPresent()) {
                robotPose = optionalRobotPose.get();
            } else {
                robotPose = new Pose2d();
            }
        field.setRobotPose(robotPose);

        String m_lastAuto = null;
        try {
            String selectedAuto = autoChooser.getSelected().getName();

            if (!DriverStation.isEnabled()) {
                // Check if the selection changed to avoid spamming NetworkTables
                if (!selectedAuto.equals(m_lastAuto)) {
                    m_lastAuto = selectedAuto;

                    // Create an empty list to populate safely
                    List<Pose2d> autoPoses = new ArrayList<>();
                    
                    // Get the path group from the auto file
                    try {
                        // This is the line that throws the exceptions
                        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(selectedAuto);
                        
                        if (paths != null) {
                            for (PathPlannerPath path : paths) {
                                autoPoses.addAll(path.getPathPoses());
                            }
                        }
                        
                    } catch (Exception e) {
                        // Catch-all for any other unhandled runtime errors
                        DriverStation.reportError("Unexpected error loading auto: " + selectedAuto, e.getStackTrace());
                    }
                    
                    // Plot onto field
                    field.getObject("Autonomous Path").setPoses(autoPoses);
                }
            }
        } catch (Exception e) {}
        
        SmartDashboard.putData("Field", field);
    }
}