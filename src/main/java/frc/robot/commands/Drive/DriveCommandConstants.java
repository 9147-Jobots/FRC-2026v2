package frc.robot.commands.Drive;

public class DriveCommandConstants {
    public static final double DEADBAND = 0.1;
    public static final double X_IN = 1;
    public static final double Y_IN = 1;
    public static final double OMEGA_IN = 0.4;

    public static final double DEFAULT_SPEED_MULTIPLIER = 0.8;
    public static final double DEFAULT_TURN_MULTIPLIER  = 1;

    public static final double SLOW_SPEED_MULTIPLIER    = 0.3;
    public static final double SLOW_TURN_MULTIPLIER     = 0.7;

    public static final double FAST_SPEED_MULTIPLIER    = 1.1;
    public static final double FAST_TURN_MULTIPLIER     = 1.0;

    // Field Y positions (meters) for bridge snap — TO BE TUNED
    public static final double SNAP_Y_BRIDGE_POS1       = 0.6;
    public static final double SNAP_Y_BRIDGE_POS2       = 7.5;

    public static final double SNAP_Y_KP                = 0.6;
}
