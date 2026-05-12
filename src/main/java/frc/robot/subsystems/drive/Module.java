package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.sensors.encoders.IEncoder;

public class Module implements IModule {

    private final IMotorVelocityControl driveMotor;
    private final IMotorPositionControl turnMotor;
    private final IEncoder turnEncoder;
    private final int index;

    private SwerveModuleState state = new SwerveModuleState();
    
    private final PIDController turnFeedback;

    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

    public Module(IMotorVelocityControl driveMotor, IMotorPositionControl turnMotor, IEncoder turnEncoder, int index) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.index = index;

        turnFeedback = new PIDController(7.0, 0.0, 0.0);

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void runSetpoint(SwerveModuleState state) {
        this.state = state;
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d(0);
        } else {
            return (new Rotation2d((turnMotor.getPosition() % 1) * 2 * Math.PI)).plus(turnRelativeOffset);
        }
    }

    public SwerveModuleState getState() {
        return this.state;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition() * 2 * Math.PI * ModuleConstants.WHEEL_RADIUS, new Rotation2d(turnEncoder.getPosition()));
    }

    @Override
    public void periodic() {
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        checkAndZeroTurnEncoder();

        if (turnRelativeOffset != null) {
            SmartDashboard.putNumber("Swerve module " + index + " angle current radians", getAngle().getRadians());
            turnMotor.runVoltage(
                turnFeedback.calculate(getAngle().getRadians(), state.angle.getRadians())
            );

            double velocityRadPerSec = state.speedMetersPerSecond / ModuleConstants.WHEEL_RADIUS;
            driveMotor.runVelocity(velocityRadPerSec);

            SmartDashboard.putNumber("Swerve module " + index + " current speed", driveMotor.getVelocity());
        }

        // LOGGING ------------------------------------------------------------------------------
        SmartDashboard.putBoolean("Swerve module " + index + " canCoderStatus:", turnRelativeOffset != null);
    }

    private void checkAndZeroTurnEncoder() {
        if (turnRelativeOffset == null && turnEncoder.getStatus().isOK()) {
            turnRelativeOffset = (new Rotation2d(turnEncoder.getPosition())).minus(new Rotation2d(turnMotor.getPosition() * 2 * Math.PI)).minus(ModuleConstants.ABSOLUTE_ENCODER_OFFSETS[index]);
        }
    }
}
