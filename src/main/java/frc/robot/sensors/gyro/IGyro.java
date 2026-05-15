package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IGyro extends SubsystemBase {
    /**
     * Returns the current angle of the gyro in radians.
     * @return The current angle of the gyro in radians.
     */
    public abstract Rotation2d getYaw();
}
