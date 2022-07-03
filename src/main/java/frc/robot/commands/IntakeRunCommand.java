package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeRunCommand extends CommandBase {
    private Intake m_intake;
    private DoubleSupplier m_speedSupplier;
    private boolean m_reversed;

    public IntakeRunCommand(Intake intake, boolean reversed, DoubleSupplier speedSupplier){
        m_intake = intake;
        m_speedSupplier = speedSupplier;
        m_reversed = reversed;

        addRequirements(m_intake);
    }

    @Override
    public void execute(){
        m_intake.runIntake(m_speedSupplier.getAsDouble());
    }

    @Override
    public void initialize(){
        m_intake.setIntakeOnly(m_reversed);
        m_intake.setReversed(m_reversed);
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopIntake();
    }
}
