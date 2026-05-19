package frc.robot.services;

import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterService {
    public static void runShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(5); // TO BE TUNED
    }

    public static void stopShooterShooter(ShooterSubsystem shooter) {
        shooter.runShooterVelocity(0);
    }

    public static void runShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerVelocity(5); // TO BE TUNED
    }

    public static void stopShooterKicker(ShooterSubsystem shooter) {
        shooter.runKickerVelocity(0);
    }    
    
    public static void TurretToBasket(ShooterSubsystem shooter) {
        // TO BE IMPLEMENTED
    }
}
