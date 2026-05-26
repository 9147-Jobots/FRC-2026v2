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

public class ShooterService {
    //create a 2d array to store the distance and angle
    private static double[][] distanceAngle = 
    {   //distance, angle
        new double[] {2.43, 2800},
        new double[] {2.85, 2850},
        new double[] {3.33, 2950},
        new double[] {3.79, 3150},
        new double[] {4.13, 3250},
        new double[] {4.33, 3350},
        new double[] {4.60, 3500},
        new double[] {4.83, 3600},
        new double[] {5.02, 3700},
        new double[] {5.30, 4200},
    };

    private static double distanceConstant = 0.2; // not entirey sure how this is meant to be set

    private static Pose2d BLUE_TARGET = new Pose2d(4.625, 4.035, new Rotation2d());
    private static Pose2d RED_TARGET = new Pose2d(11.920, 4.035, new Rotation2d());

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
    
    public static void shootFuel(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        // try {
        //     double targetRPM;
        //     SmartDashboard.putBoolean("Is in middle", isInMiddle());
        //     if (isInMiddle()) {
        //         targetRPM = getShootAreaRpm();
        //     } else {
        //         targetRPM = getTargetRpm(getDistance());
        //     }
        //     SmartDashboard.putNumber("TargetRPM", targetRPM);
        //     if (targetRPM == -1) {
        //         return;
        //     }
        //     shooter.runShooterVelocity(targetRPM);
            
        //     if (Math.abs(shooter.getShooterVelocity() - targetRPM) > 200) {
        //         shooter.runKickerVelocity(0);
        //         indexer.runVelocity(0);
        //         return;
        //     }

        //     shooter.runKickerVelocity(30);
        //     indexer.runVelocity(20);

        // } catch (Exception e) {
        //     shooter.runKickerVelocity(0);
        //     indexer.runVelocity(0);
        // }
    }

    public static Pose2d getGhostPosition(ChassisSpeeds speed, Pose2d currentPose) {
        Transform2d Displacement = new Transform2d(speed.vxMetersPerSecond * distanceConstant, speed.vyMetersPerSecond * distanceConstant, new Rotation2d(0));
        return currentPose.plus(Displacement);
    }

    public double[] getAngleVoltage() {
        return new double[] {};
    }

    public static Pose2d getTargetPosition() {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return BLUE_TARGET;
        } else {
            return RED_TARGET;
        }
    }

    public static double getTargetRpm(double distance) {
        if (distance < distanceAngle[0][0]) {
            SmartDashboard.putBoolean("Interpolation Found", false);
            return -1;
        }
        for (int i = 0; i < distanceAngle.length; i++) {
            if (distance < distanceAngle[i][0]) {
                double t = (distance - distanceAngle[i-1][0])/(distanceAngle[i][0] - distanceAngle[i-1][0]);
                SmartDashboard.putBoolean("Interpolation Found", true);
                return distanceAngle[i-1][1] + t*(distanceAngle[i][1]-distanceAngle[i-1][1]);
            }
        }
        SmartDashboard.putBoolean("Interpolation Found", false);
        return -1;
    }

    public static double getShootAreaRpm() {
        return 3000;
    }

//     public static Pose2d getTurretPose(CommandSwerveDrivetrain drive) {
//         Pose2d robotPose = drive.;
//         Pose2d robotRelativeTurretPose = new Pose2d(Constants.ShooterConstants.Turrent_x, Constants.ShooterConstants.Turrent_y, new Rotation2d());
//         Pose2d rotatedTurretPose = robotRelativeTurretPose.rotateAround(new Translation2d(), rawGyroRotation);
//         SmartDashboard.putNumber("turretrel robot x", rotatedTurretPose.getX());
//         SmartDashboard.putNumber("turretrel robot y", rotatedTurretPose.getY());
//         Transform2d movement = new Transform2d(
//         rotatedTurretPose.getTranslation(),
//         new Rotation2d()
//         );
//     return robotPose.transformBy(movement);
//   }

//     private static double getDistance() {
//         try {
//             Pose2d turrentPose = Drive.getTurretPose();
//             Pose2d targetPose = getTargetPosition();
//             Pose2d relativePose = targetPose.relativeTo(new Pose2d(turrentPose.getTranslation(), new Rotation2d()));
//             return relativePose.getTranslation().getNorm();
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//         return 0;
//     }

//     private static double getAngle() {
//         try {
//             Pose2d turrentPose = Drive.getTurretPose();
//             Pose2d targetPose = getTargetPosition();
//             Pose2d relativePose = targetPose.relativeTo(new Pose2d(turrentPose.getTranslation(), new Rotation2d()));
//             return Math.atan2(relativePose.getY(), relativePose.getX())*180/Math.PI;
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//         return 0;
//     } 

//     public static void aimTurret(Shooter shooter) {
//         try {
//             double angle = getAngle();
//             shooter.runTurretPosition(angle);
//         } catch (Exception e) {
//             // TODO: handle exception
//         }
//     }

//     public static void aimTurretArea(Shooter shooter) {
//         try {
//             double angle = getAngleToArea();
//             shooter.runTurretPosition(angle);
//         } catch (Exception e) {
//             return;
//         }
//     }

//     public static void shootFuel(Shooter shooter, Indexer indexer) {
        
//     }

//     private static double getAngleToArea() {
//         if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
//             return 0;
//         } else {
//             return 180;
//         }
//     }

//     public static void shootFuelArea(Shooter shooter, Indexer indexer) {
//         try {
//             double targetRPM = getShootAreaRpm();
//             shooter.runShooterVelocity(targetRPM);
//             if (Math.abs(shooter.getShooterVelocityRPM() - targetRPM) > 200) {
//                 shooter.stopKicker();
//                 indexer.stop();
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
