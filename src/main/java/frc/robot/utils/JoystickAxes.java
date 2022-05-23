package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.utils.Utils.deadzoneEquations;

public class JoystickAxes extends Trigger {
    private GenericHID m_controller;
    private int m_xPort;
    private int m_yPort;
    private double m_deadzone;

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
        return getDeadzonedMagnitude() != 0;
    }

    public double getDeadzonedMagnitude(){
        double mag = Math.hypot(getRawXAxis(), getRawYAxis());
        return deadzoneEquations(mag, m_deadzone);
    }

    private double getAngle(){
        return Math.atan2(getRawYAxis(), getRawXAxis());
    }

    public double getDeadzonedX(){
        return getDeadzonedMagnitude() * Math.cos(getAngle());
    }

    public double getDeadzonedY(){
        return getDeadzonedMagnitude() * Math.sin(getAngle());
    }
}
