package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeSetupCommand extends InstantCommand {
    public IntakeSetupCommand(Intake intake, boolean backwards){
        super(
            () -> {
                intake.setReversed(backwards);
                intake.setIntakeOnly(backwards);
            }, intake
        );
    }
}
