package frc.robot.sensors.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {
    /**
     * Returns the current angle of the gyro in degrees.
     * @return The current angle of the gyro in degrees.
     */
    public Rotation2d getYaw();
}
