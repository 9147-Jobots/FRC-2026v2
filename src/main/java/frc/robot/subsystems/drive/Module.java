package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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

    private Rotation2d angleSetpoint = new Rotation2d();
    private double speedSetpoint = 0.0;
    
    private final PIDController turnFeedback;

    private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute

    private SimpleMotorFeedforward ffModel =
        new SimpleMotorFeedforward(DriveConstants.driveKs, DriveConstants.driveKv); // TODO: replace

    public Module(IMotorVelocityControl driveMotor, IMotorPositionControl turnMotor, IEncoder turnEncoder, int index) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.turnEncoder = turnEncoder;
        this.index = index;

        turnFeedback = new PIDController(7.0, 0.0, 0.0);

        turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void runSetpoint(SwerveModuleState state) {
        angleSetpoint = state.angle;
        speedSetpoint = state.speedMetersPerSecond;
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d(turnMotor.getPosition() * 2 * Math.PI);
        } else {
            return (new Rotation2d(turnMotor.getPosition() * 2 * Math.PI)).plus(turnRelativeOffset);
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(speedSetpoint, angleSetpoint);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition() * ModuleConstants.WHEEL_RADIUS, new Rotation2d(turnEncoder.getPosition()));
    }

    @Override
    public void periodic() {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(getAngle());

        // On first cycle, reset relative turn encoder
        // Wait until absolute angle is nonzero in case it wasn't initialized yet
        checkAndZeroTurnEncoder();

        if (angleSetpoint != null && turnRelativeOffset != null) {
            SmartDashboard.putNumber("Swerve module " + index + " angle set point radians", angleSetpoint.getRadians());
            SmartDashboard.putNumber("Swerve module " + index + " angle current radians", getAngle().getRadians());
            turnMotor.runVoltage(
                turnFeedback.calculate(getAngle().getRadians(), angleSetpoint.getRadians())
            );
            driveMotor.runVelocity(speedSetpoint, ffModel.calculate(speedSetpoint)); // TODO: move ff into pid
        }

        // LOGGING ------------------------------------------------------------------------------
        SmartDashboard.putBoolean("Swerve module " + index + " canCoderStatus:", turnRelativeOffset != null);
    }

    private void checkAndZeroTurnEncoder() {
        if (turnRelativeOffset == null && turnEncoder.getStatus().isOK()) {
            turnRelativeOffset = (new Rotation2d(turnEncoder.getPosition())).minus(getAngle()).minus(ModuleConstants.ABSOLUTE_ENCODER_OFFSETS[index]);
        }
    }
}
