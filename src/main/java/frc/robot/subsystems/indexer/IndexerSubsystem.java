package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;

public class IndexerSubsystem {
    // int deviceID,
    // MotorType type,
    // boolean isInverted,
    // double positionConversionFactor,
    // double velocityConversionFactor,
    // double kCruiseVelocity,
    // double kMaxAcceleration,
    // double kAllowedProfileError,
    // double kP,
    // double kS,
    // double kV,
    // double kA
    IMotorVelocityControl motor;

    public IndexerSubsystem(SparkMaxVelocityControl indexer) {
        motor = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            16,
            MotorType.kBrushless,
            true,
            1,
            1,
            400,
            1000,
            0.1,
            0,
            0,
            1,
            0);
    }

    public void runVelocity(double value) {
        motor.runVelocity(value);
    }
}
