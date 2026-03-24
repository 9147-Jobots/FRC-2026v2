package frc.robot.sensors.gyro;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.RobotConstants;

public class GyroPigeon2 implements IGyro {
    private final Pigeon2 pigeon;

    public GyroPigeon2(int deviceID) {
        this.pigeon = new Pigeon2(deviceID);
        pigeon.getYaw().setUpdateFrequency(RobotConstants.Odometry.ODOMETRY_FREQUENCY);
        // Initialize the Pigeon 2 gyro with the specified CAN ID
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
    }

    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble();
    }
}
