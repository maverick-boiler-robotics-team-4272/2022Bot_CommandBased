package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRunCommand extends CommandBase {
    private Intake m_intake;
    private DoubleSupplier m_speedSupplier;

    public IntakeRunCommand(Intake intake, DoubleSupplier speedSupplier){
        m_intake = intake;
        m_speedSupplier = speedSupplier;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_intake.runIntake(m_speedSupplier.getAsDouble());
    }
}
