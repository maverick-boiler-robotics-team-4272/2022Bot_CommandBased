package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootBackCommand extends SequentialCommandGroup {
    public ShootBackCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        addCommands(
            new PathFollowCommand(
                PathPlanner.loadPath("ShootNBack", Drivetrain.MAX_LINEAR_SPEED, Drivetrain.MAX_LINEAR_ACC), 
                new PIDController(0.05, 0.0, 0.0), 
                new PIDController(0.05, 0.0, 0.0), 
                new ProfiledPIDController(0.25, 0.0, 0.0, new TrapezoidProfile.Constraints(
                    Drivetrain.MAX_ANGULAR_SPEED, 
                    Drivetrain.MAX_ANGULAR_ACC
                )), drivetrain
            ),
            new AimShootCommand(shooter, intake, drivetrain)
        );
    }
}
