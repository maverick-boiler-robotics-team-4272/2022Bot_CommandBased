package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AimShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.commands.auto_commands.AutoUtils.Trajectories.BACK_SHOOT_PATHS;

public class BackShootCommand extends SequentialCommandGroup {
    public BackShootCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        addCommands(
            new PathFollowCommand(
                BACK_SHOOT_PATHS[0],
                drivetrain
            ),
            new AimShootCommand(shooter, intake, drivetrain)
        );
    }
}
