package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sensors.gyro.GyroPigeon2;
import frc.robot.sensors.gyro.IGyro;

public class Drive extends SubsystemBase {

    private final DriveModuleGroup driveModuleGroup;
    private final IGyro gyro;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    
    private double speedMultiplier;
    private double turnMultiplier;
    
    public Drive() {
        this.driveModuleGroup = new DriveModuleGroup(
            ModuleTalon.create(ModuleConstants.driveTalonID[0], ModuleConstants.turnTalonID[0], ModuleConstants.CANcoderID[0], 0),
            ModuleTalon.create(ModuleConstants.driveTalonID[1], ModuleConstants.turnTalonID[1], ModuleConstants.CANcoderID[1], 1),
            ModuleTalon.create(ModuleConstants.driveTalonID[2], ModuleConstants.turnTalonID[2], ModuleConstants.CANcoderID[2], 2),
            ModuleTalon.create(ModuleConstants.driveTalonID[3], ModuleConstants.turnTalonID[3], ModuleConstants.CANcoderID[3], 3)
        );
        this.gyro = GyroPigeon2.CreateGyroPigeon2(DriveConstants.gyroID);

        kinematics = new SwerveDriveKinematics(DriveModuleGroup.getModuleTranslations());

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyro.getYaw(),
            driveModuleGroup.getModulePositions(),
            new Pose2d()
        );

        buildAutoBuilder();

        setSpeedMultiplier(1);
        setTurnMultiplier(1);
    }

    private void buildAutoBuilder() {
        
        // Configure AutoBuilder for PathPlanner
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = new RobotConfig(
                0d,
                0d, 
                new ModuleConfig(
                    DriveConstants.TRACK_WIDTH_Y, 
                    DriveConstants.TRACK_WIDTH_X, 
                    DriveConstants.MAX_LINEAR_SPEED, 
                    DCMotor.getKrakenX60(4), 
                    DriveConstants.MAX_ANGULAR_SPEED, 
                    DriveConstants.DRIVE_BASE_RADIUS,
                    1), 
                    DriveModuleGroup.getModuleTranslations()
                );
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            (speeds, feedforwards) -> {
                speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
                driveModuleGroup.runVelocityRaw(speeds);
            },
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                DriveConstants.TRANSLATION_PID_CONSTANTS,
                DriveConstants.ROTATION_PID_CONSTANTS
                ),
            config,
            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
            },
            this
        );
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

     public void runVelocity(ChassisSpeeds speeds) {
        //ChassisSpeeds result = applyMaxAcceleration(speeds);

        speeds.vxMetersPerSecond     *= speedMultiplier;
        speeds.vyMetersPerSecond     *= speedMultiplier;
        speeds.omegaRadiansPerSecond *= turnMultiplier;

        driveModuleGroup.runVelocityRaw(speeds);
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public void resetPose(Pose2d newPose) {
        poseEstimator.resetPosition(gyro.getYaw(), driveModuleGroup.getModulePositions(), newPose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(driveModuleGroup.getModuleStates());
    }

    private void updateOdometry() {
        poseEstimator.update(gyro.getYaw(), driveModuleGroup.getModulePositions());
    }

    @Override
    public void periodic() {
        driveModuleGroup.periodic();

        updateOdometry();
    }
    
    public void setSpeedMultiplier(double value) {
        this.speedMultiplier = value;
    }

    public void setTurnMultiplier(double value) {
        this.turnMultiplier = value;
    }
}
