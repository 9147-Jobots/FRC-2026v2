package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface IModule {

    /**
     * Runs the module at the desired velocity.
     * @param state The desired state of the module.
     */
    public void runSetpoint(SwerveModuleState state);

    /**
     * Returns the current angle of the module.
     * @return The current angle of the module.
     */
    public Rotation2d getAngle();


    /**
     * Returns the current state of the module.
     * @return The current state of the module.
     */
    public SwerveModuleState getState();

    /**
     * Returns the current position of the module, including both drive distance and turn angle.
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition();

    /**
     * Periodic method to be called in the main robot loop. Used for logging and other periodic tasks.
     */
    public void periodic();
}
