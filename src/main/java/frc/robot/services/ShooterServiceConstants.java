package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterServiceConstants {
    //create a 2d array to store the distance and angle
    public static double[][] distanceAngle = 
    {   //distance, angle
        new double[] {2.43, 2800},
        new double[] {2.85, 2850},
        new double[] {3.33, 2950},
        new double[] {3.79, 3150},
        new double[] {4.13, 3250},
        new double[] {4.33, 3350},
        new double[] {4.60, 3500},
        new double[] {4.83, 3600},
        new double[] {5.02, 3700},
        new double[] {5.30, 4200},
    };

    public static double distanceConstant = 0.2; // not entirey sure how this is meant to be set

    public static Pose2d BLUE_TARGET = new Pose2d(4.625, 4.035, new Rotation2d());
    public static Pose2d RED_TARGET = new Pose2d(11.920, 4.035, new Rotation2d());

    public static double KICKER_RUN_VELOCITY = 30;

    public static final double Turrent_x = -0.183;
    public static final double Turrent_y = 0.012;
}
