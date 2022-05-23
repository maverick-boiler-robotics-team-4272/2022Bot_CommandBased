package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickTrigger extends Trigger {
    private GenericHID m_controller;
    private int m_port;
    private double m_deadzone;

    public JoystickTrigger(GenericHID controller, int triggerNumber, double deadzone){
        m_controller = controller;
        m_port = triggerNumber;
        m_deadzone = deadzone;
    }

    @Override
    public boolean get(){
        return getDeadzonedValue() != 0.0;
    }

    public double getRawValue(){
        return m_controller.getRawAxis(m_port);
    }

    public double getDeadzonedValue(){
        return Utils.deadzoneEquations(getRawValue(), m_deadzone);
    }
}
