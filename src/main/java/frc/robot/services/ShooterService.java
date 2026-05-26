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
    public static void runShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(5); // TO BE TUNED
    }

    public static void stopShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(0);
    }

    public static void runShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerVelocity(5); // TO BE TUNED
    }

    public static void stopShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerVelocity(0);
    }    
    
//     public static void shootFuel(ShooterSubsystem shooter, IndexerSubsystem indexer) {
//         try {
//             double targetRPM;
//             SmartDashboard.putBoolean("Is in middle", isInMiddle());
//             if (isInMiddle()) {
//                 targetRPM = getShootAreaRpm();
//             } else {
//                 targetRPM = getTargetRpm(getDistance());
//             }
//             SmartDashboard.putNumber("TargetRPM", targetRPM);
//             if (targetRPM == -1) {
//                 return;
//             }
//             shooter.runShooterVelocity(targetRPM);
            
//             if (Math.abs(shooter.getShooterVelocity() - targetRPM) > 200) {
//                 shooter.runKickerVelocity(0);
//                 indexer.runVelocity(0);
//                 return;
//             }

//             shooter.runKickerVelocity(30);
//             indexer.runVelocity(20);

//         } catch (Exception e) {
//             shooter.runKickerVelocity(0);
//             indexer.runVelocity(0);
//         }
//     }

//     public static Pose2d getGhostPosition(ChassisSpeeds speed, Pose2d currentPose) {
//         Transform2d Displacement = new Transform2d(speed.vxMetersPerSecond * ShooterServiceConstants.distanceConstant, speed.vyMetersPerSecond * distanceConstant, new Rotation2d(0));
//         return currentPose.plus(Displacement);
//     }

//     public double[] getAngleVoltage() {
//         return new double[] {};
//     }

//     public static Pose2d getTargetPosition() {
//         if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
//             return ShooterServiceConstants.BLUE_TARGET;
//         } else {
//             return ShooterServiceConstants.RED_TARGET;
//         }
//     }

//     public static double getTargetRpm(double distance) {
//         if (distance < ShooterServiceConstants.distanceAngle[0][0]) {
//             SmartDashboard.putBoolean("Interpolation Found", false);
//             return -1;
//         }
//         for (int i = 0; i < ShooterServiceConstants.distanceAngle.length; i++) {
//             if (distance < ShooterServiceConstants.distanceAngle[i][0]) {
//                 double t = (distance - ShooterServiceConstants.distanceAngle[i-1][0])/(ShooterServiceConstants.distanceAngle[i][0] - ShooterServiceConstants.distanceAngle[i-1][0]);
//                 SmartDashboard.putBoolean("Interpolation Found", true);
//                 return ShooterServiceConstants.distanceAngle[i-1][1] + t*(ShooterServiceConstants.distanceAngle[i][1]-ShooterServiceConstants.distanceAngle[i-1][1]);
//             }
//         }
//         SmartDashboard.putBoolean("Interpolation Found", false);
//         return -1;
//     }

//     public static double getShootAreaRpm() {
//         return 3000;
//     }

//     public static Pose2d getTurretPose(CommandSwerveDrivetrain drive) {
//         Optional<Pose2d> optionalRobotPose = drive.samplePoseAt(UtilsJNI.getCurrentTimeSeconds());

//         Pose2d robotPose;
//         if (optionalRobotPose.isPresent()) {
//             robotPose = optionalRobotPose.get();
//         } else {
//             robotPose = new Pose2d();
//         }

//         Pose2d robotRelativeTurretPose = new Pose2d(ShooterServiceConstants.Turrent_x, ShooterServiceConstants.Turrent_y, new Rotation2d());
//         Pose2d rotatedTurretPose = robotRelativeTurretPose.rotateAround(new Translation2d(), drive.getRotation3d().toRotation2d());
//         SmartDashboard.putNumber("turretrel robot x", rotatedTurretPose.getX());
//         SmartDashboard.putNumber("turretrel robot y", rotatedTurretPose.getY());
//         Transform2d movement = new Transform2d(
//         rotatedTurretPose.getTranslation(),
//         new Rotation2d()
//         );
//         return robotPose.transformBy(movement);
//     }

//     private static double getDistance(CommandSwerveDrivetrain drive) {
//         try {
//             Pose2d turrentPose = getTurretPose(drive);
//             Pose2d targetPose = getTargetPosition();
//             Pose2d relativePose = targetPose.relativeTo(new Pose2d(turrentPose.getTranslation(), new Rotation2d()));
//             return relativePose.getTranslation().getNorm();
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//         return 0;
//     }

//     private static double getAngle(CommandSwerveDrivetrain drive) {
//         try {
//             Pose2d turrentPose = getTurretPose(drive);
//             Pose2d targetPose = getTargetPosition();
//             Pose2d relativePose = targetPose.relativeTo(new Pose2d(turrentPose.getTranslation(), new Rotation2d()));
//             return Math.atan2(relativePose.getY(), relativePose.getX())*180/Math.PI;
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//         return 0;
//     } 

//     private static void aimTurret(ShooterSubsystem shooter, CommandSwerveDrivetrain drive) {
//         try {
//             double angle = getAngle(drive);
//             shooter.runTurretPosition(angle);
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//     }

//     public static void aimTurretArea(ShooterSubsystem shooter) {
//         try {
//             double angle = getAngleToArea();
//             shooter.runTurretPosition(angle);
//         } catch (Exception e) {
//             return;
//         }
//     }

// //     public static void shootFuel(Shooter shooter, Indexer indexer) {
        
// //     }

//     private static double getAngleToArea() {
//         if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
//             return 0;
//         } else {
//             return 180;
//         }
//     }

//     public static void shootFuelArea(ShooterSubsystem shooter, IndexerSubsystem indexer) {
//         try {
//             double targetRPM = getShootAreaRpm();
//             shooter.runShooterVelocity(targetRPM);
//             if (Math.abs(shooter.getShooterVelocity() - targetRPM) > 200) {
//                 stopShooterKicker(shooter);;
//                 indexer.runVelocity(0);
//                 return;
//             }

//             shooter.runKickerDutyCycle();
//             indexer.runIndexer();
//         } catch (Exception e) {
//             shooter.stopKicker();
//             indexer.stop();
//         }
//     }

//     private static boolean isInMiddle() {
//         if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
//             if (Drive.getPose().getMeasureX().magnitude() > 4.5) {
//                 return true;
//             } else {
//                 return false;
//             }
//         } else {
//             if (Drive.getPose().getMeasureX().magnitude() < 12) {
//                 return true;
//             } else {
//                 return false;
//             }
//         }
//     }

//     public static double getAlignmentAngle() {
//         if (isInMiddle()) {
//             return getAngleToArea();
//         }
//         return getAngle();
//     }

//     public static void AlignTurretPeriodic(ShooterIO shooter) {
//         double angle = getAlignmentAngle();
//         SmartDashboard.putNumber("turret target angle", angle);
//         SmartDashboard.putNumber("turret actual angle", shooter.getTurretPosition());
//         shooter.setTurretPosition(angle, 0);
//     }
// }

// }
}
