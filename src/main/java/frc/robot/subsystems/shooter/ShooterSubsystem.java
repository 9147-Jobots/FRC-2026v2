package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;
import frc.robot.motors.tuning.VelocityGainsTuner;

public class ShooterSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // HARDCODED TURRET CONSTANTS (from old ShooterConstants.java)
    // -------------------------------------------------------------------------
    private static final int     TURRET_CAN_ID         = 9;
    private static final boolean TURRET_INVERTED        = true;
    private static final double  TURRET_VOLTAGE_COMP    = 12.0;
    private static final int     TURRET_CURRENT_LIMIT   = 40;
    private static final double  TURRET_POS_CONV_FACTOR = 9;           // CONVERSION_FACTOR from old code
    private static final double  TURRET_VEL_CONV_FACTOR = 1;           // RPM, unchanged from old code
    private static final double  TURRET_KP              = 0.1;         // TURRET_PID_MODES[0]
    private static final double  TURRET_KV              = 0.0;         // TURRET_KV
    private static final double  TURRET_CRUISE_VEL      = 1000;        // TURRET_MAX_VELOCITY (RPM)
    private static final double  TURRET_MAX_ACCEL       = 60000;       // TURRET_MAX_ACCELERATION (RPM/s)
    private static final double  TURRET_ALLOWED_ERROR   = 1.0;
    public  static final double  TURRET_OFFSET          = -177.0;      // TURRENT_OFFSET from old ShooterIOSparkMax
    // -------------------------------------------------------------------------

    IMotorVelocityControl shooter;
    IMotorVelocityControl kicker;
    VelocityGainsTuner kicker_tuner;

    // Turret: direct SparkMax (mirrors old ShooterIOSparkMax)
    private final SparkMax turret_motor;
    private final RelativeEncoder turret_encoder;
    private final SparkClosedLoopController turret_pid;

    private double turret_kP = TURRET_KP;
    private double turret_kV = TURRET_KV;

    public ShooterSubsystem() {
        shooter = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            14,
            MotorType.kBrushless,
            false,
            9,
            1,
            6000,
            2500,
            0.1,
            0.0004,
            0,
            0.00203,
            0);

        kicker = SparkMaxVelocityControl.CreateSparkMaxVelocityController(
            12,
            MotorType.kBrushless,
            true,
            9,
            1,
            1000,
            3000,
            0.1,
            0,
            0.15,
            0.00208,
            0);

        kicker_tuner = new VelocityGainsTuner("Shooter/Kicker", kicker, 0, 0.15, 0.00208, 0);

        // --- Turret: direct SparkMax setup (matches old ShooterIOSparkMax) ---
        turret_motor   = new SparkMax(TURRET_CAN_ID, MotorType.kBrushless);
        turret_encoder = turret_motor.getEncoder();
        turret_pid     = turret_motor.getClosedLoopController();

        SparkMaxConfig turret_config = new SparkMaxConfig();
        turret_config
            .inverted(TURRET_INVERTED)
            .voltageCompensation(TURRET_VOLTAGE_COMP)
            .smartCurrentLimit(TURRET_CURRENT_LIMIT)
            .encoder
                .positionConversionFactor(TURRET_POS_CONV_FACTOR)
                .velocityConversionFactor(TURRET_VEL_CONV_FACTOR);

        turret_config.closedLoop
            .p(TURRET_KP)
            .i(0)
            .d(0)
            .maxMotion
                .cruiseVelocity(TURRET_CRUISE_VEL)
                .maxAcceleration(TURRET_MAX_ACCEL)
                .allowedProfileError(TURRET_ALLOWED_ERROR);
        turret_config.closedLoop.feedForward
            .kV(TURRET_KV);

        turret_motor.configure(turret_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turret_encoder.setPosition(TURRET_OFFSET);

        // Inputs (writable from SmartDashboard)
        SmartDashboard.putNumber("Shooter/Turret/kP",          turret_kP);
        SmartDashboard.putNumber("Shooter/Turret/kV",          turret_kV);
        SmartDashboard.putNumber("Shooter/Turret/testSetpoint", TURRET_OFFSET);
        SmartDashboard.putBoolean("Shooter/Turret/isTuning",   false);
    }

    public void runShooterVelocity(double value) {
        shooter.runVelocity(value);
    }

    public double getShooterVelocity() {
        return shooter.getVelocity();
    }

    public void runKickerVelocity(double value) {
        kicker.runVelocity(value);
    }

    public double getKickerVelocity() {
        return kicker.getVelocity();
    }

    public void runTurretPosition(double degrees) {
        turret_pid.setSetpoint(degrees, ControlType.kMAXMotionPositionControl);
    }

    public double getTurretPosition() {
        return turret_encoder.getPosition();
    }

    @Override
    public void periodic() {
        kicker_tuner.update();

        // Turret gain tuning via SmartDashboard
        double newKP = SmartDashboard.getNumber("Shooter/Turret/kP", turret_kP);
        double newKV = SmartDashboard.getNumber("Shooter/Turret/kV", turret_kV);
        if (Math.abs(newKP - turret_kP) > 1e-6 || Math.abs(newKV - turret_kV) > 1e-6) {
            turret_kP = newKP;
            turret_kV = newKV;
            SparkMaxConfig update = new SparkMaxConfig();
            update.closedLoop.p(turret_kP).feedForward.kV(turret_kV);
            turret_motor.configure(update, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        boolean isTuning = SmartDashboard.getBoolean("Shooter/Turret/isTuning", false);
        if (isTuning) {
            double setpoint = SmartDashboard.getNumber("Shooter/Turret/testSetpoint", TURRET_OFFSET);
            runTurretPosition(setpoint);
        }

        // Outputs (read-only telemetry)
        SmartDashboard.putNumber("Shooter/Turret/position",      turret_encoder.getPosition());
        SmartDashboard.putNumber("Shooter/Turret/velocity",      turret_encoder.getVelocity());
        SmartDashboard.putNumber("Shooter/Turret/setpoint",      turret_pid.getSetpoint());
        SmartDashboard.putNumber("Shooter/Turret/error",         turret_pid.getSetpoint() - turret_encoder.getPosition());
        SmartDashboard.putNumber("Shooter/Turret/outputVolts",   turret_motor.getAppliedOutput() * turret_motor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Turret/currentAmps",   turret_motor.getOutputCurrent());
    }
}