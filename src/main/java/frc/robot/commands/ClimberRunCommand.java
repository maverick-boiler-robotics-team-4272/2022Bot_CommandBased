package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberRunCommand extends CommandBase {
    private Climber m_subsystem;
    private DoubleSupplier m_leftAmount;
    private DoubleSupplier m_rightAmount;

    public ClimberRunCommand(Climber climber, DoubleSupplier leftAmount, DoubleSupplier rightAmount){
        m_subsystem = climber;
        m_leftAmount = leftAmount;
        m_rightAmount = rightAmount;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute(){
        m_subsystem.runClimbers(m_leftAmount.getAsDouble(), m_rightAmount.getAsDouble());
    }

    @Override
    public void end(boolean interrupted){
        m_subsystem.runClimbers(0.0, 0.0);
    }
}
