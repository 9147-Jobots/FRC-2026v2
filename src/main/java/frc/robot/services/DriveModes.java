package frc.robot.services;

import frc.robot.subsystems.drive.Drive;


public final class DriveModes {
    public static void setDefaultMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.defaultSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.defaultTurnMutliplier);
    }
    public static void setSlowMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.slowSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.slowTurnMutliplier);
    }
    
    public static void setFastMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.fastSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.fastTurnMutliplier);
    }
}
