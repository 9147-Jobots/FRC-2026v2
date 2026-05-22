package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;

public class IndexerSubsystem extends SubsystemBase {
    IMotorVelocityControl motor;

    public IndexerSubsystem() {
        motor = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            16,
            MotorType.kBrushless,
            true,
            1,
            1,
            400,
            1000,
            0.1,
            0.0004,
            0,
            0.00206,
            0);
    }

    public void runVelocity(double value) {
        motor.runVelocity(value);
    }
}
