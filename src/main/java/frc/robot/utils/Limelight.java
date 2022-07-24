package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    private static final double LIMELIGHT_DEADZONE = 4.0;
    private static final double LIMELIGHT_GROUND_ANGLE = 45;
    private static final double GOAL_HEIGHT = 8.5;
    private static final double LIMELIGHT_HEIGHT = 2.75;

    //Limelight calibration

    //place robot, and then measure from limelight for these values
    private static final double PHI_A = 4.48;
    private static final double PHI_B = -7.85;
    private static final double PHI_C = -15.41;

    //tuned hood angles

    public static double THETA_A = -12.0;
    public static double THETA_B = -15.8;
    public static double THETA_C = -17.25;

    //tuned flywheel speeds
    public static double OMEGA_A = 1925.0;
    public static double OMEGA_B = 2200.0;
    public static double OMEGA_C = 2400.0;


    //y = ax^2 + bx + c
    //a, b, and c values

    public static final double HOOD_ANGLE_A =   THETA_A / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - THETA_B / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C / ((PHI_A - PHI_C) * (PHI_B - PHI_C));
    public static final double HOOD_ANGLE_B = - THETA_A * (PHI_B + PHI_C) / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) + THETA_B * (PHI_A + PHI_C) / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C * (PHI_A + PHI_B)/ ((PHI_A - PHI_C) * (PHI_C - PHI_B));
    public static final double HOOD_ANGLE_C =   THETA_A * PHI_B * PHI_C / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - THETA_B * PHI_A * PHI_C / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + THETA_C * PHI_A * PHI_B / ((PHI_A - PHI_C) * (PHI_B - PHI_C));

    public static final double FLYWHEEL_SPEED_A =   OMEGA_A / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - OMEGA_B / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C / ((PHI_A - PHI_C) * (PHI_B - PHI_C));
    public static final double FLYWHEEL_SPEED_B = - OMEGA_A * (PHI_B + PHI_C) / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) + OMEGA_B * (PHI_A + PHI_C) / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C * (PHI_A + PHI_B) / ((PHI_A - PHI_C) * (PHI_C - PHI_B));
    public static final double FLYWHEEL_SPEED_C =   OMEGA_A * PHI_B * PHI_C / ((PHI_A - PHI_B) * (PHI_A - PHI_C)) - OMEGA_B * PHI_A * PHI_C / ((PHI_A - PHI_B) * (PHI_B - PHI_C)) + OMEGA_C * PHI_A * PHI_B / ((PHI_A - PHI_C) * (PHI_B - PHI_C));


    public enum LEDMode {
        PIPELINE_CURRENT(0),
        OFF(1),
        BLINK(2),
        ON(3);

        public final int m_mode;
        private LEDMode(int mode){
            m_mode = mode;
        }
    }

    private Limelight() {}

    public void setLEDMode(LEDMode mode){
        table.getEntry("ledMode").setNumber(mode.m_mode);
    }

    /**
     * Gets the x angle of the target. Because our limelight is rotated 90
     * degrees, x angle is the ty value
     * @return limelight ty value
     */
    public double getTX(){
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the y angle of the target. Because out limelight is rotated 90
     * degrees, y angle is the tx value
     * @return limelight tx
     */
    public double getTY(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getDistance(){
        return (GOAL_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan((LIMELIGHT_GROUND_ANGLE + getTY()) * Constants.DEG_TO_RAD);
    }

    public boolean getValidTarget(){
        return ((int) table.getEntry("tv").getNumber(0)) != 0;
    }

    public boolean getAimed(){
        return Math.abs(getTX()) < LIMELIGHT_DEADZONE && getValidTarget();
    }

    public double getFlywheelSpeed(){
        double ty = getTY();
        return FLYWHEEL_SPEED_A * ty * ty + FLYWHEEL_SPEED_B * ty + FLYWHEEL_SPEED_C;
    }

    public double getHoodAngle(){
        double ty = getTY();
        return HOOD_ANGLE_A * ty * ty + HOOD_ANGLE_B * ty + HOOD_ANGLE_C;
    }
}
