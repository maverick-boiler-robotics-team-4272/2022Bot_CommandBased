package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

public class Terminal2Ball extends AutoCommand {
    public Terminal2Ball(Drivetrain drivetrain){
        super(
            PathPlanner.loadPath("TestPath", Drivetrain.MAX_LINEAR_ACC, Drivetrain.MAX_ANGULAR_ACC),
            drivetrain
        );

        addCommands(
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                getStopCommand(),
                new WaitCommand(1),
                getStartCommand()
            )
        );
    }
}
