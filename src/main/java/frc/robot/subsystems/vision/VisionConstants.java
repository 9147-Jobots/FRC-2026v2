package frc.robot.subsystems.vision;

public class VisionConstants {

    public static final class CameraLeft {
        public static final double X     = -0.273; // Uncalibrated
        public static final double Y     =  0.252; // Uncalibrated
        public static final double Z     =  0.235; // Uncalibrated
        public static final double PITCH = 10 * Math.PI / 180; // Uncalibrated
        public static final double ROLL  = 0; // Uncalibrated
        public static final double YAW   = 169.5 * Math.PI / 180; // Uncalibrated
    }

    public static final class CameraRight {
        public static final double X     = -0.27; // Uncalibrated
        public static final double Y     = -0.249; // Uncalibrated
        public static final double Z     =  0.235; // Uncalibrated
        public static final double PITCH = 10 * Math.PI / 180; // Uncalibrated
        public static final double ROLL  = 180 * Math.PI / 180;
        public static final double YAW   = 190.5 * Math.PI / 180; // Uncalibrated
    }
}
