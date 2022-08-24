package frc.robot.commands.auto_commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BackShootCommand extends AutoCommand {
    public BackShootCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        super(
            "ShootNBack",
            shooter,
            intake,
            drivetrain
        );
/*
        addCommands(
            new SequentialCommandGroup(
                new WaitUntilCommand(getFollowCommand()::isFinished)//,
                //new AimShootCommand(shooter, intake, drivetrain)
            )
        );*/
    }
}
