package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeSetup extends InstantCommand {
    public IntakeSetup(Intake intake, boolean backwards){
        super(
            () -> {
                intake.setReversed(backwards);
                intake.setIntakeOnly(backwards);
            }, intake
        );
    }
}
