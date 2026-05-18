package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.motors.SparkMax.SparkMaxVelocityControl;

public class Indexer {
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
    SparkMaxVelocityControl motor;

    public Indexer(SparkMaxVelocityControl indexer) {
        motor = SparkMaxVelocityControl.CreateSparkMaxVelocityontroller();
    }
}
