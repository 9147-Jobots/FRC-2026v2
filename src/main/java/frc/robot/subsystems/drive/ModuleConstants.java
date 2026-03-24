package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ModuleConstants {
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0;

    public static final Rotation2d[] ABSOLUTE_ENCODER_OFFSETS = {Rotation2d.fromDegrees(-56.513671875),  
                                                                 Rotation2d.fromDegrees(45.087890625),
                                                                 Rotation2d.fromDegrees(-4.21875),
                                                                 Rotation2d.fromDegrees(122.6953125)};

    // CAN IDs for drive motors, turn motors and absolute encoders respectively. Order is FL, FR, BL, BR.
    public static final int[] driveTalonID = {0, 6, 9, 3};
    public static final int[] turnTalonID = {1, 7, 10, 4};
    public static final int[] CANcoderID = {2, 8, 11, 5};

    // Whether drive and turn motors should be inverted. Order is FL, FR, BL, BR.
    public static final Boolean[] isTurnMotorInverted = {true, true, false, false};
    public static final Boolean[] isDriveMotorInverted = {false, false, true, true};

    public static final double TURN_SUPPLY_CURRENT_LIMIT = 30;
    public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 40;

    public static final Boolean TURN_SUPPLY_CURRENT_ENABLE = true;
    public static final Boolean DRIVE_SUPPLY_CURRENT_ENABLE = true;

    public static final Boolean TURN_BRAKE_MODE = true;
    public static final Boolean DRIVE_BRAKE_MODE = true;
}
