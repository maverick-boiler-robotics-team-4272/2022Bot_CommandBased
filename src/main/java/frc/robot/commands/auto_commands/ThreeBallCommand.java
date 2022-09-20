package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.commands.auto_commands.AutoUtils.Trajectories.THREE_BALL_PATHS;

public class ThreeBallCommand extends SequentialCommandGroup {
    public ThreeBallCommand(Shooter shooter, Intake intake, Drivetrain drivetrain) {
        addCommands(
            new PathFollowCommand(
                THREE_BALL_PATHS[0],
                drivetrain
            )
        );
    }
}
