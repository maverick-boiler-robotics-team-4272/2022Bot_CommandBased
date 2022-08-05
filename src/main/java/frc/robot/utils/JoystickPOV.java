package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class JoystickPOV extends Trigger {
    private GenericHID m_controller;
    private final int m_povPort;

    public JoystickPOV(GenericHID controller, int povPort){
        m_controller = controller;
        m_povPort = povPort;
    }

    public JoystickPOV(GenericHID controller){
        this(controller, 0);
    }

    @Override
    public boolean get() {
        return getValue() >= 0;
    }

    public int getValue(){
        return m_controller.getPOV(m_povPort);
    }
}
