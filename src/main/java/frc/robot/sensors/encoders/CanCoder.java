package frc.robot.sensors.encoders;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class CanCoder implements IEncoder {

    private final CANcoder canCoder;

    public CanCoder(int id) {
        this.canCoder = new CANcoder(id);
        canCoder.getConfigurator().apply(new CANcoderConfiguration());
    }

    @Override
    public double getPosition() {
        StatusSignal<Angle> positionSignal = canCoder.getAbsolutePosition();
        return positionSignal.getValue().in(Units.Radians);
    }

    @Override
    public double getVelocity() {
        StatusSignal<AngularVelocity> velocitySignal = canCoder.getVelocity();
        return velocitySignal.getValue().in(Units.RadiansPerSecond);
    }

    @Override
    public StatusCode getStatus() {
        return canCoder.getAbsolutePosition().getStatus();
    }

    @Override
    public void reset() {
        canCoder.setPosition(0);
    }
    
}
