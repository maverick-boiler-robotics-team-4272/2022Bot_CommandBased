package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;

public class Limelight {
    private static final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
    private static final ShuffleboardTable m_sTable = ShuffleboardTable.getTable("Shooter");

    private static final double LIMELIGHT_DEADZONE = 4.0;
    private static final double LIMELIGHT_GROUND_ANGLE = 45;
    private static final double GOAL_HEIGHT = 8.5;
    private static final double LIMELIGHT_HEIGHT = 2.75;

    //Limelight calibration

    //place robot, and then measure from limelight for these values
    private static double PHI_A = 4.48;
    private static double PHI_B = -7.85;
    private static double PHI_C = -15.41;

    //tuned hood angles

    private static double THETA_A = -12.0;
    private static double THETA_B = -15.8;
    private static double THETA_C = -17.25;

    //tuned flywheel speeds
    private static double OMEGA_A = 1925.0;
    private static double OMEGA_B = 2200.0;
    private static double OMEGA_C = 2400.0;

    private static Calibration hoodCalibration = new Calibration(PHI_A, PHI_B, PHI_C, THETA_A, THETA_B, THETA_C);
    private static Calibration speedCalibration = new Calibration(PHI_A, PHI_B, PHI_C, OMEGA_A, OMEGA_B, OMEGA_C);

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

    public static void setLEDMode(LEDMode mode){
        m_table.getEntry("ledMode").setNumber(mode.m_mode);
    }

    /**
     * Gets the x angle of the target. Because our limelight is rotated 90
     * degrees, x angle is the ty value
     * @return limelight ty value
     */
    public static double getTX(){
        return m_table.getEntry("ty").getDouble(0.0);
    }

    /**
     * Gets the y angle of the target. Because out limelight is rotated 90
     * degrees, y angle is the tx value
     * @return limelight tx
     */
    public static double getTY(){
        return m_table.getEntry("tx").getDouble(0.0);
    }

    /**
     * Computes the distance to the goal on the XY plane
     * @return
     */
    public static double getDistance(){
        return (GOAL_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan((LIMELIGHT_GROUND_ANGLE + getTY()) * Constants.DEG_TO_RAD);
    }

    /**
     * @return Whether or not the limelight is aimed at a valid target
     */
    public static boolean getValidTarget(){
        return ((int) m_table.getEntry("tv").getNumber(0)) != 0;
    }

    /**
     * @return Whether or not the limelight is aimed at the goal
     */
    public boolean getAimed(){
        return Math.abs(getTX()) < LIMELIGHT_DEADZONE && getValidTarget();
    }

    /**
     * Computes a flywheel speed based on three prededtermined points
     * @return flywheel speed
     */
    public double getFlywheelSpeed(){
        return speedCalibration.getCalibratedValue(getTY());
    }

    /**
     * Computes a hood angle based on three predetermined points
     * @return hood angle
     */
    public double getHoodAngle(){
        return hoodCalibration.getCalibratedValue(getTY());
    }

    /**
     * Updates the calibration values with values from Shuffleboard
     */
    public void updateCalibration(){
        PHI_A = m_sTable.getNumber("Phi A");
        PHI_B = m_sTable.getNumber("Phi B");
        PHI_C = m_sTable.getNumber("Phi C");

        THETA_A = m_sTable.getNumber("Theta A");
        THETA_B = m_sTable.getNumber("Theta B");
        THETA_C = m_sTable.getNumber("Theta C");

        OMEGA_A = m_sTable.getNumber("Omega A");
        OMEGA_B = m_sTable.getNumber("Omega B");
        OMEGA_C = m_sTable.getNumber("Omega C");

        hoodCalibration.setValues(PHI_A, PHI_B, PHI_C, THETA_A, THETA_B, THETA_C);
        speedCalibration.setValues(PHI_A, PHI_B, PHI_C, OMEGA_A, OMEGA_B, OMEGA_C);
    }

    /**
     * Initial setup of the dashboard
     */
    public void setupDashboard(){
        m_sTable.putNumber("Phi A", PHI_A).withPosition(0, 0);
        m_sTable.putNumber("Phi B", PHI_B).withPosition(0, 1);
        m_sTable.putNumber("Phi C", PHI_C).withPosition(0, 2);

        m_sTable.putNumber("Theta A", THETA_A).withPosition(1, 0);
        m_sTable.putNumber("Theta B", THETA_B).withPosition(1, 1);
        m_sTable.putNumber("Theta C", THETA_C).withPosition(1, 2);

        m_sTable.putNumber("Omega A", OMEGA_A).withPosition(2, 0);
        m_sTable.putNumber("Omega B", OMEGA_B).withPosition(2, 1);
        m_sTable.putNumber("Omega C", OMEGA_C).withPosition(2, 2);

        m_sTable.putData("Update", new InstantCommand(this::updateCalibration)).withPosition(1, 3);

        m_sTable.putNumber("Phi", getTY()).withPosition(4, 0);
    }
}
