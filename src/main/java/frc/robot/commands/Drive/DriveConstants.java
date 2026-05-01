package frc.robot.commands.Drive;

import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public static final double TRACK_WIDTH_Y = 0.686;
    public static final double TRACK_WIDTH_X = 0.686;
    public static final double DRIVE_BASE_RADIUS =
      Math.hypot(DriveConstants.TRACK_WIDTH_X / 2.0, DriveConstants.TRACK_WIDTH_Y / 2.0);

    public static final double DEADBAND = 0.05;
    public static final double X_IN = 1;
    public static final double Y_IN = 1;
    public static final double OMEGA_IN = 0.4;
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(15);
    public static final double MAX_ANGULAR_SPEED = DriveConstants.MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
    
}
