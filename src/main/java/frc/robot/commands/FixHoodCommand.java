package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.ShooterConstants.ShooterPositions;

public class FixHoodCommand extends CommandBase {
    private Shooter m_shooter;
    private CANSparkMax m_motor;

    public FixHoodCommand(Shooter shooter){
        m_shooter = shooter;
        m_motor = shooter.getHoodMotor();

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        m_motor.set(0.05);
    }

    @Override
    public void end(boolean interrupted) {
        m_motor.getEncoder().setPosition(0.0);
        m_motor.set(0.0);
        m_shooter.setShooter(ShooterPositions.EJECT);
    }

    @Override
    public boolean isFinished() {
        return m_motor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
    }
}
