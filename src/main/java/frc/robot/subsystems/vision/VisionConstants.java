package frc.robot.subsystems.vision;

public class VisionConstants {

    // Robot: 68.8cm long (X), 68.5cm wide (Y). Origin at center.
    // All positions measured from physical robot; Z heights uncalibrated.

    public static final class CameraLeft {
        public static final double X     = -0.2665; // 3.05in from back edge
        public static final double Y     =  0.2600; // 3.25in from left edge
        public static final double Z     =  0.07;   // 4cm bottom, 10cm top → center 7cm
        public static final double PITCH = 10  * Math.PI / 180; // 10deg up
        public static final double ROLL  = 0;
        public static final double YAW   = 145 * Math.PI / 180; // backward, 35deg toward left
    }

    public static final class CameraRight {
        public static final double X     = -0.2691; // 2.95in from back edge
        public static final double Y     = -0.2663; // 3in from right edge
        public static final double Z     =  0.07;   // 4cm bottom, 10cm top → center 7cm
        public static final double PITCH = 10  * Math.PI / 180; // 10deg up
        public static final double ROLL  = 0;
        public static final double YAW   = 215 * Math.PI / 180; // backward, 35deg toward right
    }
}
