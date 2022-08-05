package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.XBoxController;

public class SetHoodCommand extends CommandBase {
    private Shooter m_shooter;
    private XBoxController m_controller;

    public SetHoodCommand(Shooter shooter, XBoxController controller){
        m_shooter = shooter;
        m_controller = controller;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setShooter(m_controller.getPOV().getValue());
    }

    @Override
    public void execute() {
        m_shooter.setHood();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.getHoodAtPosition();
    }
}
