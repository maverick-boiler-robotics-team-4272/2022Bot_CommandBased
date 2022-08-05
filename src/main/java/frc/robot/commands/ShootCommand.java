package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
    protected Shooter m_shooter;
    protected Intake m_intake;

    public ShootCommand(Shooter shooter, Intake intake){
        m_shooter = shooter;
        m_intake = intake;

        addRequirements(shooter, intake);
    }

    @Override
    public void execute() {
        m_shooter.shoot();
    }

    @Override
    public boolean isFinished() {
        return !m_intake.ballPresent();
    }
}
