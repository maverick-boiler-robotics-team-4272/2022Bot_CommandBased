package frc.robot.commands.ClimberCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberRun extends CommandBase {
    private Climber m_subsystem;
    private DoubleSupplier m_leftAmount;
    private DoubleSupplier m_rightAmount;

    public ClimberRun(Climber climber, DoubleSupplier leftAmount, DoubleSupplier rightAmount){
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
    public boolean isFinished(){
        return false;
    }
}
