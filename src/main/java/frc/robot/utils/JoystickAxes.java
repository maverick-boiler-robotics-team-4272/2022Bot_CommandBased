package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.utils.Utils.deadzoneEquations;

public class JoystickAxes extends Trigger {
    enum DeadzoneMode{
        MAGNITUDE,
        X_AXIS,
        Y_AXIS
    }

    private GenericHID m_controller;
    private int m_xPort;
    private int m_yPort;
    private double m_deadzone;

    private DeadzoneMode m_mode = DeadzoneMode.MAGNITUDE;

    public JoystickAxes(GenericHID controller, int xAxis, int yAxis, double deadzone){
        m_controller = controller;
        m_xPort = xAxis;
        m_yPort = yAxis;
        m_deadzone = deadzone;
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
                val = getRawDeadzonedX();
                break;
            case Y_AXIS:
                val = getRawDeadzonedY();
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

    public double getDeadzonedMagnitude(){
        double mag = Math.hypot(getRawXAxis(), getRawYAxis());
        return deadzoneEquations(mag, m_deadzone);
    }

    private double getAngle(){
        return Math.atan2(getRawYAxis(), getRawXAxis());
    }

    /**
     * 
     * @return deadzoned x taking into acount circular deadzone
     */
    public double getDeadzonedX(){
        return getDeadzonedMagnitude() * Math.cos(getAngle());
    }

    /**
     * 
     * @return deadzoned y taking into acount circular deadzone
     */
    public double getDeadzonedY(){
        return getDeadzonedMagnitude() * Math.sin(getAngle());
    }

    /**
     * 
     * @return deadzoned x, only x axis deadzoned
     */
    public double getRawDeadzonedX(){
        return deadzoneEquations(getRawXAxis(), m_deadzone);
    }

    /**
     * 
     * @return deadzoned y, only y axis deadzoned
     */
    public double getRawDeadzonedY(){
        return deadzoneEquations(getRawYAxis(), m_deadzone);
    }
}
