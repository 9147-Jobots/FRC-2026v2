package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    // Measurements for Transform3D
    public final class cameraRight { // 0.27 meters from the center backwards, 0.249 meters to the left, 0.235 meters up, 0.26 radians pitch, 0.0 radians roll
        public static final double x = -0.27;
        public static final double y = -0.249;
        public static final double z = 0.235;
        public static final double pitch = 10*Math.PI/180; //15 degrees
        public static final double roll = 0;
        public static final double yaw = (190.5)*Math.PI/180;
    }
    public final class cameraLeft { // 0.273 meters from the center backwards, 0.252 meters to the right, 0.235 meters up, 0 radians pitch, 0.0 radians roll
        public static final double x = -0.273;
        public static final double y = 0.252;
        public static final double z = 0.235;
        public static final double pitch = 10*Math.PI/180;
        public static final double roll = 0;
        public static final double yaw = (169.5)*Math.PI/180;
    }

    public final class PoseEstimatorConstants {
        public static final Matrix<N3, N1> stateStdDevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{0.1, 0.1, 0.1});
    }
  }