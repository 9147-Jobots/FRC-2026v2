package frc.robot.sensors.encoders;

import com.ctre.phoenix6.StatusCode;

/**
 * Interface for all external encoders (not built into motor controllers) used in the robot.
 */
public interface IEncoder {
    /**
     * Returns the current position of the encoder in rads.
     * @return the current position of the encoder in rads
     */
    public double getPosition();

    /**
     * Returns the current velocity of the encoder in rads per second.
     * @return the current velocity of the encoder in rads per second
     */
    public double getVelocity();

     /**
      * Resets the encoder's position to zero.
      */
    public void reset();

    public StatusCode getStatus();
}
