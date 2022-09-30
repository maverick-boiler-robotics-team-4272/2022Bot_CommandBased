package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveCommand extends CommandBase {
    private Drivetrain m_drivetrain;
    private DoubleSupplier m_xSpeed;
    private DoubleSupplier m_ySpeed;
    private DoubleSupplier m_rSpeed;

    private SlewRateLimiter m_xLimiter = new SlewRateLimiter(MAX_ACCELERATION);
    private SlewRateLimiter m_yLimiter = new SlewRateLimiter(MAX_ACCELERATION);
    private SlewRateLimiter m_thetaLimiter = new SlewRateLimiter(MAX_ACCELERATION);

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotationSpeed){
        m_drivetrain = drivetrain;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_rSpeed = rotationSpeed;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute(){
        m_drivetrain.drive(m_xLimiter.calculate(m_xSpeed.getAsDouble() * MAX_SPEED), m_yLimiter.calculate(m_ySpeed.getAsDouble() * MAX_SPEED), m_thetaLimiter.calculate(m_rSpeed.getAsDouble() * MAX_SPEED));
    }

    @Override
    public void end(boolean interrupted){
        m_drivetrain.drive(0.0, 0.0, 0.0);
    }
}