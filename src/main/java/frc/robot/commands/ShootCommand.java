package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
    protected Shooter m_shooter;
    protected Intake m_intake;
    private Timer m_timer = new Timer();
    private boolean m_started;

    public ShootCommand(Shooter shooter, Intake intake){
        m_shooter = shooter;
        m_intake = intake;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize(){
        m_timer.stop();
        m_timer.reset();
        m_started = false;
    }

    @Override
    public void execute() {
        m_shooter.shoot();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
        m_intake.stopIntake();
        m_intake.stopFeedShooter();
    }

    @Override
    public boolean isFinished() {
        if(!m_intake.ballPresent()){
            if(!m_started) {
                m_timer.start();
                m_started = true;
            }

            return m_timer.hasElapsed(1.0);
        }
        return false;
    }
}
