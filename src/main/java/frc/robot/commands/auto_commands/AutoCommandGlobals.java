package frc.robot.commands.auto_commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain;

public class AutoCommandGlobals {
    private AutoCommandGlobals() {};

    public static final PIDController X_PID_CONTROLLER = new PIDController(0.2, 0.01, 0);
    public static final PIDController Y_PID_CONTROLLER = new PIDController(0.2, 0.01, 0);
    public static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(4.5, 0, 0,
    new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_SPEED, Drivetrain.MAX_ANGULAR_ACC));
}
