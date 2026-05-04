package frc.robot.services;

import frc.robot.subsystems.drive.Drive;
import frc.robot.services.DriveModesConstants;




public final class DriveModes {
    public void setDefaultMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.defaultSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.defaultTurnMutliplier);
    }
    public void setSlowMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.slowSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.slowTurnMutliplier);
    }
    
    public void setFastMode(Drive drive) {
        drive.setSpeedMultiplier(DriveModesConstants.fastSpeedMutliplier);
        drive.setTurnMultiplier(DriveModesConstants.fastTurnMutliplier);
    }
}
