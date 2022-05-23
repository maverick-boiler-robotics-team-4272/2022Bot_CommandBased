package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.*;

public class Utils {
    public static double deadzoneEquations(double input, double deadzone){
        if(input >= deadzone){
            return (input - deadzone) / (1.0 - deadzone);
        }else if(input <= -deadzone){
            return (input + deadzone) / (1.0 - deadzone);
        }else{
            return 0.0;
        }
    }

    public static void noop(){

    }

    public static JoystickButton joystickButton(XboxController controller, XboxController.Button button){
        return new JoystickButton(controller, button.value);
    }

    public static JoystickTrigger joystickTrigger(XboxController controller, XboxController.Axis axis){
        return new JoystickTrigger(controller, axis.value, TRIGGER_DEADZONE);
    }

    public static JoystickAxes joystickAxes(XboxController controller, XboxController.Axis xAxis, XboxController.Axis yAxis){
        return new JoystickAxes(controller, xAxis.value, yAxis.value, JOYSTICK_DEADZONE);
    }
}
