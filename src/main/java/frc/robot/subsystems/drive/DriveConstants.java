package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public static final double driveKs = 0; // to be tunned static friction voltage in volts
    public static final double driveKv = 1.98; // to be tunned velocity constant in volts per (meter per second)

    // Gyro constants
    public static final int gyroID = 12;

    // Input constants
    public static final double X_IN = 1;
    public static final double Y_IN = 1;
    public static final double OMEGA_IN = 0.4;

    /** m/s */
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15); //TODO up it
    public static final double TRACK_WIDTH_Y = 0.686;
    public static final double TRACK_WIDTH_X = 0.686;
    public static final double DRIVE_BASE_RADIUS =
      Math.hypot(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED =  DriveConstants.MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    //MoveToConstants
    public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(0.5, 0.0, 0.1); //TODO auto
    public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(0.8, 0.0, 0.0); //TODO auto

    public static final double linearK = 0.5;
    public static final double AngularK = 0.5;

    public static double loopPeriodSecs = 0.02;
    public static double turnDeadbandDegrees = 5.0;

    public class ModuleConstants {
        public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
        public static final double ODOMETRY_FREQUENCY = 250.0;
    }
}
