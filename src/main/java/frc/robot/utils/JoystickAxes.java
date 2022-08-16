package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.utils.Utils.deadzoneEquations;

public class JoystickAxes extends Trigger {
    public enum DeadzoneMode{
        MAGNITUDE,
        X_AXIS,
        Y_AXIS
    }

    private GenericHID m_controller;
    private int m_xPort;
    private int m_yPort;
    private double m_deadzone;
    private double m_power = 1;

    private DeadzoneMode m_mode = DeadzoneMode.MAGNITUDE;

    public JoystickAxes(GenericHID controller, int xAxis, int yAxis, double deadzone){
        m_controller = controller;
        m_xPort = xAxis;
        m_yPort = yAxis;
        m_deadzone = deadzone;
    }

    private double powerScale(double input){
        return Math.signum(input) * Math.pow(input, m_power);
    }

    public double getRawXAxis(){
        return m_controller.getRawAxis(m_xPort);
    }

    public double getRawYAxis(){
        return m_controller.getRawAxis(m_yPort);
    }

    @Override
    public boolean get(){
        double val;

        switch(m_mode){
            case MAGNITUDE:
                val = getDeadzonedMagnitude();
                break;
            case X_AXIS:
                val = getDeadzonedX();
                break;
            case Y_AXIS:
                val = getDeadzonedY();
                break;
            default:
                val = 0.0;
                break;
        }

        return val != 0.0;
    }

    /**
     * Sets the deadzoning mode for triggering the command
     * 
     * @param mode
     * 
     * @apiNote MAGNITUDE deadzones magnitude
     * @apiNote X_AXIS deadzones soley the X axis
     * @apiNote Y_AXIS deadzones soley the Y axis
     */
    public void setMode(DeadzoneMode mode) {
        m_mode = mode;
    }

    /**
     * @return Current deadzoning mode
     */
    public DeadzoneMode getMode(){
        return m_mode;
    }

    public double getPowerScaling(){
        return m_power;
    }

    public void setPowerScaling(double power){
        m_power = power;
    }

    public double getDeadzonedMagnitude(){
        double mag = Math.hypot(getRawXAxis(), getRawYAxis());
        mag = deadzoneEquations(mag, m_deadzone);
        return powerScale(mag);
    }

    private double getAngle(){
        return Math.atan2(getRawYAxis(), getRawXAxis());
    }

    /**
     * 
     * @return deadzoned x value
     * @apiNote Deadzones based off of the deadzoning mode
     * @apiNote If deadzoning mode is Y_AXIS, this returns 0.0
     */
    public double getDeadzonedX(){
        switch(m_mode){
            case MAGNITUDE:
                return getDeadzonedMagnitude() * Math.cos(getAngle());
            case X_AXIS:
                return powerScale(deadzoneEquations(getRawXAxis(), m_deadzone));
            default:
                return 0.0;
        }
    }

    /**
     * 
     * @return deadzoned y value
     * @apiNote Deadzones based off of the deadzoning mode
     * @apiNote If deadzoning mode is X_AXIS, this returns 0.0
     */
    public double getDeadzonedY(){
        switch(m_mode){
            case MAGNITUDE:
                return getDeadzonedMagnitude() * Math.sin(getAngle());
            case Y_AXIS:
                return powerScale(deadzoneEquations(getRawYAxis(), m_deadzone));
            default:
                return 0.0;
        }
    }
}
