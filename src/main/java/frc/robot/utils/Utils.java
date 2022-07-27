package frc.robot.utils;

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
}
