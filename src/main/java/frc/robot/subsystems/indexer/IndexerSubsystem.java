package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;
import frc.robot.motors.tuning.VelocityGainsTuner;

public class IndexerSubsystem extends SubsystemBase {
    IMotorVelocityControl motor;
    private final VelocityGainsTuner tuner;

    public IndexerSubsystem() {
        motor = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            16,
            MotorType.kBrushless,
            false,
            1,
            1,
            400,
            1000,
            0.1,
            0,
            0.4,
            0.0022,
            0);

        tuner = new VelocityGainsTuner("Indexer/Motor", motor, 0.0004, 0, 0.00206, 0);
    }

    @Override
    public void periodic() {
        //tuner.update();
    }

    public void runVelocity(double value) {
        motor.runVelocity(value);
    }

    public void runDutyCycle(double value) {
        motor.runDutyCycle(value);
    }
}
