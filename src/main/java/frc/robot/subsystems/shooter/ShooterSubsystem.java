package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMotorPositionControl;
import frc.robot.motors.IMotorPositionControl.PositionControlMode;
import frc.robot.motors.IMotorVelocityControl;
import frc.robot.motors.SparkMax.SparkMaxPositionControl;
import frc.robot.motors.SparkMax.SparkMaxVelocityControl;
import frc.robot.motors.tuning.PositionGainsTuner;
import frc.robot.motors.tuning.VelocityGainsTuner;
import frc.robot.services.ShooterService;

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
    private static final double  TURRET_KP              = 0.05;         // TURRET_PID_MODES[0]
    private static final double  TURRET_KV              = 0.0;         // TURRET_KV
    private static final double  TURRET_CRUISE_VEL      = 10000;        // TURRET_MAX_VELOCITY (RPM)
    private static final double  TURRET_MAX_ACCEL       = 60000;       // TURRET_MAX_ACCELERATION (RPM/s)
    private static final double  TURRET_ALLOWED_ERROR   = 1.0;
    public  static final double  TURRET_OFFSET          = -187.0 + 10;      // TURRENT_OFFSET from old ShooterIOSparkMax
    // -------------------------------------------------------------------------

    IMotorVelocityControl shooter;
    IMotorVelocityControl kicker;
    IMotorPositionControl turret;
    VelocityGainsTuner kicker_tuner;
    PositionGainsTuner turret_tuner;

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

        turret  = SparkMaxPositionControl.CreateLinearSparkMaxPositionController(
            TURRET_CAN_ID,
            MotorType.kBrushless,
            TURRET_INVERTED,
            TURRET_POS_CONV_FACTOR,
            TURRET_VEL_CONV_FACTOR,
            TURRET_CRUISE_VEL,
            TURRET_MAX_ACCEL,
            TURRET_ALLOWED_ERROR,
            TURRET_KP,
            0,
            TURRET_KV,
            0,
            -0.2,
            0.2);
            
        turret_tuner = new PositionGainsTuner("Shooter/Turret", turret, TURRET_KP, 0, TURRET_KV, 0, 0, 0);
        turret.zeroPosition(TURRET_OFFSET);
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

    public void runKickerDutyCycle(double value) {
        kicker.runDutyCycle(value);
    }

    public double getKickerVelocity() {
        return kicker.getVelocity();
    }

    public void runTurretPosition(double degrees) {
        while (degrees < -340) {
            degrees += 360;}
            
        while (degrees > 50) {
            degrees -= 360;}

        turret.runPosition(degrees, PositionControlMode.kDirectPID);
    }

    public double getTurretPosition() {
        return turret.getPosition();
    }

    @Override
    public void periodic() {
        //kicker_tuner.update();
        //turret_tuner.update();
    }
}