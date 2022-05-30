package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberPneumaticsToggle extends CommandBase {
    Climber m_subsystem;
    public ClimberPneumaticsToggle(Climber climber){
        m_subsystem = climber;

        addRequirements(m_subsystem);
    }

    @Override
    public void execute(){
        m_subsystem.toggleClimber();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
