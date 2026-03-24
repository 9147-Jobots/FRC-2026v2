package frc.robot.subsystems.drive;

import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.sensors.encoders.IEncoder;

/**
 * This module implements 2 CAN Talon FXs for the drive and turn motors, and a CAN CANCoder for the turn encoder.
 */
public class ModuleTalon extends Module {
    private ModuleTalon(IMotorVelocityControl driveMotor, IMotorPositionControl turnMotor, IEncoder turnEncoder, int index) {
        super(driveMotor, turnMotor, turnEncoder, index);
    }

    public static ModuleTalon create(int driveMotorID, int turnMotorID, int turnEncoderID, int index) {
        
        IMotorVelocityControl driveMotor = null; // Replace with actual motor creation logic
        IMotorPositionControl turnMotor = null; // Replace with actual motor creation logic
        IEncoder turnEncoder = null; // Replace with actual encoder creation logic
        return new ModuleTalon(driveMotor, turnMotor, turnEncoder, index);
    }
}
