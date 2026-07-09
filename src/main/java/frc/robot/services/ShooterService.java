package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.jni.UtilsJNI;

public class ShooterService {
    public static void runIndexer(IndexerSubsystem indexer) {
        indexer.runDutyCycle(0.8);
    }

    public static void stopIndexer(IndexerSubsystem indexer) {
        indexer.runVelocity(0); // TO BE TUNED
    }

    public static void runShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(5); // TO BE TUNED
    }

    public static void stopShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(0);
    }

    public static void runShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerDutyCycle(1);
    }

    public static void stopShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerVelocity(0);
    }    
    
    public static void shootFuel(ShooterSubsystem shooter, IndexerSubsystem indexer, CommandSwerveDrivetrain drive) {
        try {
            double targetRPM;
            SmartDashboard.putBoolean("Is in middle", isInMiddle(drive));
            if (isInMiddle(drive)) {
                targetRPM = getShootAreaRpm();
            } else {
                targetRPM = getTargetRpm(getDistanceToTarget(drive));
            }
            SmartDashboard.putNumber("TargetRPM", targetRPM);
            if (targetRPM == -1) {
                return;
            }
            shooter.runShooterVelocity(targetRPM);
            
            if (Math.abs(shooter.getShooterVelocity() - targetRPM) > 200) {
                runShooterKicker(shooter);
                stopIndexer(indexer);
                return;
            }

            runIndexer(indexer);
            runShooterKicker(shooter);

        } catch (Exception e) {
            stopShooterKicker(shooter);
            stopIndexer(indexer);
        }
    }

    public static Pose2d getGhostPosition(ChassisSpeeds speed, Pose2d currentPose) {
        Transform2d Displacement = new Transform2d(speed.vxMetersPerSecond * ShooterServiceConstants.distanceConstant, speed.vyMetersPerSecond * ShooterServiceConstants.distanceConstant, new Rotation2d(0));
        return currentPose.plus(Displacement);
    }

    public double[] getAngleVoltage() {
        return new double[] {};
    }

    public static Pose2d getTargetPosition() {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return ShooterServiceConstants.BLUE_TARGET;
        } else {
            return ShooterServiceConstants.RED_TARGET;
        }
    }

    public static double getTargetRpm(double distance) {
        if (distance < ShooterServiceConstants.distanceAngle[0][0]) {
            SmartDashboard.putBoolean("Interpolation Found", true);
            return ShooterServiceConstants.distanceAngle[0][1];
        }
        for (int i = 0; i < ShooterServiceConstants.distanceAngle.length; i++) {
            if (distance < ShooterServiceConstants.distanceAngle[i][0]) {
                double t = (distance - ShooterServiceConstants.distanceAngle[i-1][0])/(ShooterServiceConstants.distanceAngle[i][0] - ShooterServiceConstants.distanceAngle[i-1][0]);
                SmartDashboard.putBoolean("Interpolation Found", true);
                return ShooterServiceConstants.distanceAngle[i-1][1] + t*(ShooterServiceConstants.distanceAngle[i][1]-ShooterServiceConstants.distanceAngle[i-1][1]) + ShooterServiceConstants.offset;
            }
        }
        SmartDashboard.putBoolean("Interpolation Found", false);
        return ShooterServiceConstants.distanceAngle[0][1];
    }

    public static double getShootAreaRpm() {
        return 3000;
    }

    public static Pose2d getTurretPose(CommandSwerveDrivetrain drive) {
        Optional<Pose2d> optionalRobotPose = drive.samplePoseAt(UtilsJNI.getCurrentTimeSeconds());

        Pose2d robotPose;
        if (optionalRobotPose.isPresent()) {
            robotPose = optionalRobotPose.get();
        } else {
            robotPose = new Pose2d();
        }

        SmartDashboard.putNumber("turretrel robot x", ShooterServiceConstants.Turrent_x);
        SmartDashboard.putNumber("turretrel robot y", ShooterServiceConstants.Turrent_y);
        // transformBy rotates the robot-local offset by robotPose.rotation into field frame
        Transform2d movement = new Transform2d(
            new Translation2d(ShooterServiceConstants.Turrent_x, ShooterServiceConstants.Turrent_y),
            new Rotation2d()
        );
        return robotPose.transformBy(movement);
    }

    public static double getDistanceToTarget(CommandSwerveDrivetrain drive) {
        try {
            Pose2d turrentPose = getTurretPose(drive);
            Pose2d targetPose = getTargetPosition();
            Pose2d relativePose = targetPose.relativeTo(turrentPose);
            return relativePose.getTranslation().getNorm();
        } catch (Exception e) {
            // TODO: handle exception
        }
        return 0;
    }

    private static double getAngle(CommandSwerveDrivetrain drive) {
        try {
            Pose2d turrentPose = getTurretPose(drive);
            Pose2d targetPose = getTargetPosition();
            Pose2d relativePose = targetPose.relativeTo(turrentPose);
            return Math.atan2(relativePose.getY(), relativePose.getX())*180/Math.PI;
        } catch (Exception e) {
            // TODO: handle exception
        }
        return 0;
    } 

    public static void aimTurret(ShooterSubsystem shooter, CommandSwerveDrivetrain drive) {
        try {
            double angle = getAngle(drive);
            shooter.runTurretPosition(angle);
        } catch (Exception e) {
            // TODO: handle exception
        }
    }

    public static void aimTurretArea(ShooterSubsystem shooter, CommandSwerveDrivetrain drive) {
        try {
            double angle = getAngleToArea(drive);
            shooter.runTurretPosition(angle);
        } catch (Exception e) {
            return;
        }
    }

    private static double getAngleToArea(CommandSwerveDrivetrain drive) {
        double fieldAngle = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red) ? 0 : 180;
        return fieldAngle - drive.getRotation3d().toRotation2d().getDegrees();
    }

    public static void shootFuelArea(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        try {
            double targetRPM = getShootAreaRpm();
            shooter.runShooterVelocity(targetRPM);
            if (Math.abs(shooter.getShooterVelocity() - targetRPM) > 200) {
                stopShooterKicker(shooter);;
                stopIndexer(indexer);
                return;
            }

            runShooterKicker(shooter);
            runIndexer(indexer);
        } catch (Exception e) {
            stopShooterKicker(shooter);
            stopIndexer(indexer);
        }
    }

    public static boolean isInMiddle(CommandSwerveDrivetrain drive) {
        Optional<Pose2d> optionalRobotPose = drive.samplePoseAt(UtilsJNI.getCurrentTimeSeconds());

        Pose2d robotPose;
        if (optionalRobotPose.isPresent()) {
            robotPose = optionalRobotPose.get();
        } else {
            robotPose = new Pose2d();
        }

        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (robotPose.getMeasureX().magnitude() > 4.5) {
                return true;
            } else {
                return false;
            }
        } else {
            if (robotPose.getMeasureX().magnitude() < 12) {
                return true;
            } else {
                return false;
            }
        }
    }

    public static double getAlignmentAngle(CommandSwerveDrivetrain drive) {
        if (isInMiddle(drive)) {
            return getAngleToArea(drive);
        }
        return getAngle(drive);
    }

    public static void AlignTurretPeriodic(ShooterSubsystem shooter, CommandSwerveDrivetrain drive) {
        double angle = getAlignmentAngle(drive);
        double robotFieldAngle = drive.getRotation3d().toRotation2d().getDegrees();
        double turretRobotAngle = shooter.getTurretPosition();
        double turretFieldAngle = robotFieldAngle + turretRobotAngle;

        SmartDashboard.putNumber("robot angle (field-rel)", robotFieldAngle);
        SmartDashboard.putNumber("turret actual angle (robot-rel)", turretRobotAngle);
        SmartDashboard.putNumber("turret angle (field-rel)", turretFieldAngle);
        SmartDashboard.putNumber("curr turret angle", angle);
        try {
            Pose2d turrentPose = getTurretPose(drive);
            Pose2d targetPose = getTargetPosition();
            double targetFieldAngle = Math.atan2(
                targetPose.getY() - turrentPose.getY(),
                targetPose.getX() - turrentPose.getX()
            ) * 180 / Math.PI;
            SmartDashboard.putNumber("target angle (field-rel)", targetFieldAngle);
        } catch (Exception e) {}
        shooter.runTurretPosition(angle);
    }
}
