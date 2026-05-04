package frc.robot.subsystems.drive;

import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.TalonFX.TalonFXPositionControl;
import frc.robot.motors.TalonFX.TalonFXVelocityControl;
import frc.robot.sensors.encoders.CanCoder;
import frc.robot.sensors.encoders.IEncoder;

/**
 * This module implements 2 CAN Talon FXs for the drive and turn motors, and a CAN CANCoder for the turn encoder.
 */
public class ModuleTalon {

    public static Module create(int driveMotorID, int turnMotorID, int turnEncoderID, int index) {
        
        IMotorVelocityControl driveMotor = TalonFXVelocityControl.CreateTalonFXVelocityControl(
            driveMotorID, 
            ModuleConstants.isDriveMotorInverted[index],
            ModuleConstants.DRIVE_GEAR_RATIO,
            ModuleConstants.DRIVE_PID.getP(),
            ModuleConstants.DRIVE_FF.getKs(),
            ModuleConstants.DRIVE_FF.getKv()
        );
        IMotorPositionControl turnMotor = TalonFXPositionControl.CreateTalonFXPositionControl(
            turnMotorID,
            ModuleConstants.isTurnMotorInverted[index],
            ModuleConstants.TURN_GEAR_RATIO,
            ModuleConstants.TURN_PID.getP(),
            ModuleConstants.TURN_PID.getI(),
            ModuleConstants.TURN_PID.getD(),
            ModuleConstants.TURN_FF.getKs()
        );
        IEncoder turnEncoder = CanCoder.CreateCanCoder(turnEncoderID);

        return new Module(driveMotor, turnMotor, turnEncoder, index);
    }
}
