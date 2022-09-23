package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ShootCommand;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.Tables.SHOOTER_TABLE;

public class Shooter extends SubsystemBase {

    private double m_hoodAmt;
    private double m_shooterAmt;
    private double m_feedAmt;

    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax m_shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax m_shooterFollowerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax m_hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController m_shooterPIDController = m_shooterMotor.getPIDController();
    private SparkMaxPIDController m_hoodPIDController = m_hoodMotor.getPIDController();

    private Intake m_intake;

    private boolean ballShooting = false;
    
    private double shooterP = 0.00004;
    private double shooterI = 0.0;
    private double shooterD = 0.0;
    private double shooterFF = 0.000199;//0.0001963

    private double hoodP = 0.00001;
    private double hoodI = 0.00000007;
    private double hoodD = 0.0;
    private double hoodFF = 0.000075;

    public Shooter(Intake intake){

        m_intake = intake;

        /*
        hoodMotor.restoreFactoryDefaults(true);
        shooterMotor.restoreFactoryDefaults(true);
        shooterFollowerMotor.restoreFactoryDefaults(true);
        */

        m_hoodMotor.getEncoder().setPositionConversionFactor(1);

        m_hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
        m_hoodPIDController.setP(hoodP);
        m_hoodPIDController.setI(hoodI);
        m_hoodPIDController.setD(hoodD);
        m_hoodPIDController.setFF(hoodFF);

        m_hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
        m_hoodPIDController.setOutputRange(-1.25, 1.25);
        m_hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        m_hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        m_hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);

        m_hoodMotor.getEncoder().setPosition(0.0);
        m_hoodMotor.setSmartCurrentLimit(40);
        m_shooterMotor.setInverted(true);
        
        m_shooterFollowerMotor.follow(m_shooterMotor, true);
        m_shooterPIDController.setP(shooterP);
        m_shooterPIDController.setI(shooterI);
        m_shooterPIDController.setD(shooterD);
        m_shooterPIDController.setFF(shooterFF);

        m_shooterPIDController.setSmartMotionMaxVelocity(3000.0, 0);
        m_shooterPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        m_shooterPIDController.setSmartMotionMaxAccel(4000.0, 0);
        m_shooterPIDController.setSmartMotionAllowedClosedLoopError(5.0, 0);

        m_shooterMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        m_shooterFollowerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);

        m_shooterMotor.setIdleMode(IdleMode.kCoast);
        m_shooterFollowerMotor.setIdleMode(IdleMode.kCoast);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);

        m_hoodMotor.burnFlash();
        m_shooterMotor.burnFlash();
        m_shooterFollowerMotor.burnFlash();

    }

    /**
     * Starts the shooter wheel based on the shooter amount variable that is determined by the dpad
     */
    public void shoot(){

        if(m_intake.getShooterBeam()){
            ballShooting = true;
        }

        if(ballShooting && !m_intake.getShooterBeam()){
            m_intake.shotBall();
            ballShooting = false;
        }

        m_shooterMotor.getPIDController().setReference(m_shooterAmt, ControlType.kSmartVelocity);

        if(Math.abs(m_shooterMotor.getEncoder().getVelocity() - m_shooterAmt) <= SHOOTER_DEADZONE &&
            getHoodAtPosition()){
                m_intake.feedShooter(m_feedAmt);
        }else{
            m_intake.stopIntake();
            m_intake.stopFeedShooter();
        }

    }

    /**
     * Stops the shooter
     */
    public void stopShooter(){
        this.m_shooterMotor.set(0);
        m_intake.stopFeedShooter();
        m_intake.stopIntake();
        m_intake.resetBall();
    }

    public CANSparkMax getHoodMotor(){
        return m_hoodMotor;
    }

    /**
     * Sets shooterAmt and hoodAmt variables to specified values in an array who's index is determined by
     * the inputted dpad value
     * 
     * @param pov pov from an XboxController
     */
    public void setShooter(int pov){
        int index = pov / 90;

        ShooterPositions[] shooterSetpoints = ShooterPositions.values();

        setShooter(shooterSetpoints[index].shootAmt, shooterSetpoints[index].hoodAmt, shooterSetpoints[index].feedAmt);

    }

    /**
     * Sets all of the parameters of the shooter
     * @param shooterAmount speed to run the shooter at
     * @param hoodAmount position to set the hood to
     * @param feedAmount speed to run the feed motor at
     */
    public void setShooter(double shooterAmount, double hoodAmount, double feedAmount){

        m_shooterAmt = shooterAmount;
        m_hoodAmt = hoodAmount;
        m_feedAmt = feedAmount;
        setHood();

    }

    /**
     * Set the position of the shooter through an enum
     * @param shooterPosition the enum
     */
    public void setShooter(ShooterPositions shooterPosition){

        setShooter(shooterPosition.shootAmt, shooterPosition.hoodAmt, shooterPosition.feedAmt);

    }

    /**
     * Revs up the shooter, but will not shoot. Used so we can shoot sooner in auto
     */
    public void revShooter(){

        m_shooterMotor.getPIDController().setReference(m_shooterAmt, ControlType.kSmartVelocity);

    }

    /**
     * Updates the shooter and hood from smart dashboard
     */
    public void updateShooter() {

        m_shooterAmt = SHOOTER_TABLE.getNumber("Shooter Velocity Set", 0.0);
        m_hoodAmt = SHOOTER_TABLE.getNumber("Hood Setpoint", 0.0);
        m_feedAmt = -0.6;

    }

    /**
     * Sets hood position
     */
    public void setHood(){
        m_hoodMotor.getPIDController().setReference(m_hoodAmt, ControlType.kSmartMotion);
    }


    /**
     * Set shooterMotor from smart dashboard
     */
    public void tuneShoot(){
        this.m_shooterMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
    }

    /**
     * Returns if the hood is within a certain deadzone of its desired position
     * @return
     */
    public boolean getHoodAtPosition(){
        double hoodPos = m_hoodMotor.getEncoder().getPosition();

        return Math.abs(hoodPos - m_hoodAmt) < HOOD_DEADZONE;

    }

    public void reBurnFlash(){

        m_hoodMotor.restoreFactoryDefaults(true);
        m_shooterMotor.restoreFactoryDefaults(true);
        m_shooterFollowerMotor.restoreFactoryDefaults(true);

        this.m_hoodMotor.getEncoder().setPositionConversionFactor(1);

        m_hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        
        m_hoodPIDController.setP(hoodP);
        m_hoodPIDController.setI(hoodI);
        m_hoodPIDController.setD(hoodD);
        m_hoodPIDController.setFF(hoodFF);

        m_hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
        m_hoodPIDController.setOutputRange(-1.25, 1.25);
        m_hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        m_hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        m_hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);

        m_hoodMotor.getEncoder().setPosition(0.0);
        m_hoodMotor.setSmartCurrentLimit(20);
        m_shooterMotor.setInverted(true);
        
        m_shooterFollowerMotor.follow(m_shooterMotor, true);
        m_shooterPIDController.setP(shooterP);
        m_shooterPIDController.setI(shooterI);
        m_shooterPIDController.setD(shooterD);
        m_shooterPIDController.setFF(shooterFF);

        m_shooterPIDController.setSmartMotionMaxVelocity(3000.0, 0);
        m_shooterPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        m_shooterPIDController.setSmartMotionMaxAccel(4000.0, 0);
        m_shooterPIDController.setSmartMotionAllowedClosedLoopError(5.0, 0);

        m_shooterMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        m_shooterFollowerMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);

        m_shooterMotor.setIdleMode(IdleMode.kCoast);
        m_shooterFollowerMotor.setIdleMode(IdleMode.kCoast);

        m_hoodMotor.burnFlash();
        m_shooterMotor.burnFlash();
        m_shooterFollowerMotor.burnFlash();

    }

    public void resetFactoryDefaults(){

        m_shooterMotor.restoreFactoryDefaults(true);
        m_shooterFollowerMotor.restoreFactoryDefaults(true);

    }
}
