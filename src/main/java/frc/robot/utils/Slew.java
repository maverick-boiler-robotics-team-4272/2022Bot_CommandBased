package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class Slew extends SlewRateLimiter {
    private double prevValue;

    public Slew(double acceleration) {
        super(acceleration);
    }

    @Override
    public double calculate(double input) {
        if(Math.signum(input) != Math.signum(prevValue)) reset(0.0);
        prevValue = input;
        return super.calculate(input);
    }
}
