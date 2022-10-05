package frc.robot.utils;

public class Utils {

    private Utils() {
        throwUtilityConstructor(this);
    }

    public static double deadzoneEquations(double input, double deadzone){
        if(input >= deadzone){
            return (input - deadzone) / (1.0 - deadzone);
        }else if(input <= -deadzone){
            return (input + deadzone) / (1.0 - deadzone);
        }else{
            return 0.0;
        }
    }

    public static double euclideanModulo(double n, double m){
        return ( (n % m) + m ) % m;
    }

    public static void noop(){

    }

    public static void throwUtilityConstructor(Object obj) {
        throw new UnsupportedOperationException(obj.getClass().getName() + " is a utility class");
    }
}
