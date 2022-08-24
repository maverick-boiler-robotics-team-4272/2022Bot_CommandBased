package frc.robot.commands.auto_commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TuneAutoCommand extends AutoCommand {
    public TuneAutoCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        super(
            "TunePath",
            shooter,
            intake,
            drivetrain
        );
    }
}
