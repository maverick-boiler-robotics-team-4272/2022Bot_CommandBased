package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;

public class XBoxController {

    private static final double JOYSTICK_DEADZONE = 0.15;
    private static final double TRIGGER_DEADZONE = 0.1;

    public enum Buttons {
        A_BUTTON,
        B_BUTTON,
        X_BUTTON,
        Y_BUTTON,

        START_BUTTON,
        BACK_BUTTON,

        LEFT_BUMPER,
        RIGHT_BUMPER,

        LEFT_STICK_CLICK,
        RIGHT_STICK_CLICK
    }

    public enum Triggers {
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    public enum Axes {
        LEFT_STICK,
        RIGHT_STICK
    }

    private XboxController m_controller;

    private JoystickButton m_aButton;
    private JoystickButton m_bButton;
    private JoystickButton m_xButton;
    private JoystickButton m_yButton;

    private JoystickButton m_startButton;
    private JoystickButton m_backButton;

    private JoystickButton m_leftBumper;
    private JoystickButton m_rightBumper;

    private JoystickButton m_leftClick;
    private JoystickButton m_rightClick;

    private JoystickTrigger m_leftTrigger;
    private JoystickTrigger m_rightTrigger;

    private JoystickAxes m_leftStick;
    private JoystickAxes m_rightStick;
    public XBoxController(int port){
        m_controller = new XboxController(port);

        m_aButton = new JoystickButton(m_controller, kA.value);
        m_bButton = new JoystickButton(m_controller, kB.value);
        m_xButton = new JoystickButton(m_controller, kX.value);
        m_yButton = new JoystickButton(m_controller, kY.value);

        m_startButton = new JoystickButton(m_controller, kStart.value);
        m_backButton = new JoystickButton(m_controller, kBack.value);

        m_leftBumper = new JoystickButton(m_controller, kLeftBumper.value);
        m_rightBumper = new JoystickButton(m_controller, kRightBumper.value);

        m_leftClick = new JoystickButton(m_controller, kLeftStick.value);
        m_rightClick = new JoystickButton(m_controller, kRightStick.value);

        m_leftTrigger = new JoystickTrigger(m_controller, kLeftTrigger.value, TRIGGER_DEADZONE);
        m_rightTrigger = new JoystickTrigger(m_controller, kRightTrigger.value, TRIGGER_DEADZONE);

        m_leftStick = new JoystickAxes(m_controller, kLeftX.value, kLeftY.value, JOYSTICK_DEADZONE);
        m_rightStick = new JoystickAxes(m_controller, kRightX.value, kRightY.value, JOYSTICK_DEADZONE);
    }

    public JoystickButton getButton(Buttons button){
        switch(button){
            case A_BUTTON:
                return m_aButton;
            case B_BUTTON:
                return m_bButton;
            case X_BUTTON:
                return m_xButton;
            case Y_BUTTON:
                return m_yButton;
            case START_BUTTON:
                return m_startButton;
            case BACK_BUTTON:
                return m_backButton;
            case LEFT_BUMPER:
                return m_leftBumper;
            case RIGHT_BUMPER:
                return m_rightBumper;
            case LEFT_STICK_CLICK:
                return m_leftClick;
            case RIGHT_STICK_CLICK:
                return m_rightClick;
            default:
                return null;
        }
    }

    public JoystickTrigger getTrigger(Triggers trigger){
        switch(trigger){
            case LEFT_TRIGGER:
                return m_leftTrigger;
            case RIGHT_TRIGGER:
                return m_rightTrigger;
            default:
                return null;
        }
    }

    public JoystickAxes getAxis(Axes axis){
        switch(axis){
            case LEFT_STICK:
                return m_leftStick;
            case RIGHT_STICK:
                return m_rightStick;
            default:
                return null;
        }
    }
}
