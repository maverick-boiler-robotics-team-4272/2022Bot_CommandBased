package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.commands.auto_commands.AutoUtils.Trajectories.TUNE_PATHS;

public class TuneAutoCommand extends SequentialCommandGroup {
    public TuneAutoCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        super(
            new PathFollowCommand(
                TUNE_PATHS[0],
                drivetrain
            )
        );
    }
}
