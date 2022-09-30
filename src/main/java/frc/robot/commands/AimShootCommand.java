package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Limelight;
import frc.robot.utils.Limelight.LEDMode;

import static frc.robot.Constants.Tables.TESTING_TABLE;

public class AimShootCommand extends ShootCommand {

    private static final PIDController DEFAULT_CONTROLLER = new PIDController(0.15, 0.0, 0.0);


    private Drivetrain m_drivetrain;
    private PIDController m_controller;

    public AimShootCommand(Shooter shooter, Intake intake, Drivetrain drivetrain, PIDController pidController){
        super(shooter, intake);

        m_drivetrain = drivetrain;
        m_controller = pidController;
        m_controller.setSetpoint(0.0);

        addRequirements(drivetrain);
    }

    public AimShootCommand(Shooter shooter, Intake intake, Drivetrain drivetrain){
        this(
            shooter,
            intake,
            drivetrain,
            DEFAULT_CONTROLLER
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        Limelight.setLEDMode(LEDMode.ON);
    }
    
    @Override
    public void execute() {
        m_shooter.setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
        m_shooter.setHood();

        TESTING_TABLE.putBoolean("Limelight Aimed", Limelight.getAimed());

        if(Limelight.getAimed()){
            m_shooter.shoot();
            m_drivetrain.xConfig();
        } else {
            m_shooter.revShooter();
            
            double rotationSpeed = m_controller.calculate(Limelight.getTX());
            m_drivetrain.drive(0.0, 0.0, rotationSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Limelight.setLEDMode(LEDMode.OFF);
        m_drivetrain.drive(0.0, 0.0, 0.0);
    }
}