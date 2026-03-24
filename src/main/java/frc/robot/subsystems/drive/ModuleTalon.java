package frc.robot.subsystems.drive;

import frc.robot.motors.IPositionControlMotor;
import frc.robot.motors.IVelocityControlMotor;
import frc.robot.sensors.encoders.IEncoder;

/**
 * This module implements 2 CAN Talon FXs for the drive and turn motors, and a CAN CANCoder for the turn encoder.
 */
public class ModuleTalon extends Module {
    private ModuleTalon(IVelocityControlMotor driveMotor, IPositionControlMotor turnMotor, IEncoder turnEncoder, int index) {
        super(driveMotor, turnMotor, turnEncoder, index);
    }

    public static ModuleTalon create(int driveMotorID, int turnMotorID, int turnEncoderID, int index) {
        
        IVelocityControlMotor driveMotor = null; // Replace with actual motor creation logic
        IPositionControlMotor turnMotor = null; // Replace with actual motor creation logic
        IEncoder turnEncoder = null; // Replace with actual encoder creation logic
        return new ModuleTalon(driveMotor, turnMotor, turnEncoder, index);
    }
}
