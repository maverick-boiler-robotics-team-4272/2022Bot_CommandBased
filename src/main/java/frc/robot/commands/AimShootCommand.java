package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LEDMode;

public class AimShootCommand extends ShootCommand {
    private Drivetrain m_drivetrain;
    private PIDController m_controller;

    public AimShootCommand(Shooter shooter, Intake intake, Drivetrain drivetrain, PIDController pidController){
        super(shooter, intake);

        m_drivetrain = drivetrain;
        m_controller = pidController;
        m_controller.setSetpoint(0.0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Limelight.setLEDMode(LEDMode.ON);
    }
    
    @Override
    public void execute() {
        m_shooter.setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
        m_shooter.setHood();

        if(Limelight.getAimed()){
            m_shooter.shoot();
        } else {
            m_shooter.revShooter();
            
            double rotationSpeed = m_controller.calculate(Limelight.getTX());
            m_drivetrain.drive(0.0, 0.0, rotationSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.setLEDMode(LEDMode.OFF);
        m_drivetrain.drive(0.0, 0.0, 0.0);
        m_shooter.stopShooter();
        m_intake.stopIntake();
        m_intake.stopFeedShooter();
    }
}