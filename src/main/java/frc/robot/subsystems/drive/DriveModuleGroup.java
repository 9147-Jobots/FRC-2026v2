package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveModuleGroup {

    private final IModule[] modules = new IModule[4]; // FL, FR, BL, BR

    private static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

    public DriveModuleGroup(
        IModule flModuleIO,
        IModule frModuleIO,
        IModule blModuleIO,
        IModule brModuleIO
    ) {
        modules[0] = flModuleIO;
        modules[1] = frModuleIO;
        modules[2] = blModuleIO;
        modules[3] = brModuleIO;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocityRaw(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, DriveConstants.loopPeriodSecs);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED);

        for (int i = 0; i < 4; i++) {
            setpointStates[i].optimize(modules[i].getAngle());
            modules[i].runSetpoint(setpointStates[i]);
            
            SmartDashboard.putNumber("Module " + i + " speed", setpointStates[i].speedMetersPerSecond);
            SmartDashboard.putNumber("Module " + i + " angle", setpointStates[i].angle.getDegrees());
        }

        // Log setpoint states
        // Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        // Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
        new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0),
        new Translation2d(-DriveConstants.TRACK_WIDTH_X / 2.0, -DriveConstants.TRACK_WIDTH_Y / 2.0)
        };
    }

    /**
     * Returns the current positions of all modules.
     * @return An array of SwerveModulePosition objects representing the current positions of all modules.
     */
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public void periodic() {
        for (int i = 0; i < 4; i++) {
            modules[i].periodic();
        }
    }
}
