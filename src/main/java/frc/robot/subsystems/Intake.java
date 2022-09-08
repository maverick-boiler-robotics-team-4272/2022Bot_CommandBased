package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Lidar;

import static frc.robot.Constants.*;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    //Intake motors, ids 7-10(if needed)
    //Beam breaks are true for unbroke and false for broken
    private DigitalInput m_shooterBeamBreak = new DigitalInput(16); //top feed
    private DigitalInput m_midFeedBeamBreak = new DigitalInput(15); //mid feed
    private DigitalInput m_lowFeedBeamBreak = new DigitalInput(14); //close to intake
    private Lidar m_hopperLidar1 = new Lidar(20);
    private Lidar m_hopperLidar2 = new Lidar(18);
    private Lidar m_hopperLidar3 = new Lidar(19);
    
    //run intake booleans
    private boolean b1 = false;
    private boolean b2 = false;
    private boolean b1InFeed = false;
    private boolean b1Prepped = false;
    private boolean b1Mid = false;

    private boolean reversed = false;
    private boolean override = false;
    private boolean intakeOnly = false;

    private double hopperUpperBound = 0.1;
    private double hopperLowerBound = 0.03;
    

    private CANSparkMax m_intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax m_shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 1, 0);

    private double currentCurrentLimit = INTAKE_NORM_CURR_LIM;

    public Intake(){
        m_intakeMotor.setSmartCurrentLimit(55);

        m_shooterFeedMotor.setSmartCurrentLimit(45);
        m_shooterFeedMotor.setOpenLoopRampRate(0.5);

        m_intakeMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        m_shooterFeedMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);

        m_intakeMotor.burnFlash();
        m_shooterFeedMotor.burnFlash();

        m_intakeSolenoid.set(Value.kForward);
    }

    /**
     * Run the intake and feed, using beam breaks to figure out when to stop
     * @param triggerVal speed to run intake at
     * @param inverted whether the feed motor is inverted
     * @param override intake override to not use hopperbeam
     * @param intakeOnly to run just the intake and not the feed
     */
    public void runIntake(double triggerVal){

        boolean midBeam = !m_midFeedBeamBreak.get();
        boolean shooterBeam = !m_shooterBeamBreak.get();
        boolean hopperBeam = getHopperBeam();
        double feedVal = -0.6;

        if(intakeOnly){
            m_intakeMotor.set(triggerVal);
            return;
        }

        if(reversed){
            m_intakeMotor.set(-triggerVal);
            m_shooterFeedMotor.set(-feedVal);
            resetBall();
            return;
        }

        if(override){

            m_intakeMotor.set(triggerVal);

            if(midBeam || shooterBeam || b1Prepped){
                m_shooterFeedMotor.set(0.0);
            }else{
                m_shooterFeedMotor.set(feedVal);
            }

            if(getIntakeLidar() && b1){
                b2 = true;
            }

            return;

        }

        //if no ball then just run norms
        //if ball in hopper, slow intake speed to allow feed succ
        //if ball in feed and not hitting mid beam, run norm speed
        //if ball in feed and hit mid beam, slow feed
        //run feed til ball hit shooter beam
        //only run intake

        //////pre 4/5 code//////
        
        if(midBeam && !b1Mid){

            b1Mid = true;

        }else if(b1Mid && midBeam){

            feedVal = -0.3;

        }else if(b1Mid && !midBeam){

            feedVal = 0;
            b1InFeed = true;

        }

        if(b1InFeed && hopperBeam){
            b2 = true;
        }

        if(hopperBeam){
            if(!b1){
                b1 = true;
            }
            triggerVal = 0.15;
            setIntakeCurrentLimit(55);
        }else{
            setIntakeCurrentLimit(45);
        }

        if(getIntakeLidar() && currentCurrentLimit < 60){
            setIntakeCurrentLimit(60);
        }

        m_intakeMotor.set(triggerVal);

        m_shooterFeedMotor.set(feedVal);
        
    }
    
    public boolean hopperFull(){
        return (b1 && b2);
    }
    

    public int getBallCount(){
        int count = 0;

        if(b1){
            count++;
        }

        if(b2){
            count++;
        }

        return count;
    }

    public void shotBall(){
        if(b1 && b2){
            b2 = false;
        }else if(b1 && !b2){
            b1 = false;
        }else if(!b1 && b2){
            b2 = false;
        }
    }

    /**
     * Reverses the feed motor
     * @param val speed to run the motor at
     */
    public void reverseFeed(double val){
        m_shooterFeedMotor.set(val);
    }

    /**
     * 
     * @return true if a ball is in the robot anywhere there's a sensor, false otherwise
     */
    public boolean ballPresent(){
        
        boolean botBeam = !m_lowFeedBeamBreak.get();
        boolean midBeam = !m_midFeedBeamBreak.get();
        boolean shooterBeam = !m_shooterBeamBreak.get();
        boolean hopper = getBallInHopper();
        
        return (botBeam || midBeam || shooterBeam || hopper || b1);

    }

    public boolean getBallInHopper(){
        return 
            getIntakeLidar() || getMidHopperLidar() || getBackHopperLidar();
    }

    /**
     * Resets all the booleans
     */
    public void resetBall(){
        b1 = false;
        b2 = false;
        b1Mid = false;
        b1Prepped = false;
        b1InFeed = false;
    }

    /**
     * Completely stops the intake
     */
    public void stopIntake(){
        m_intakeMotor.set(0.0);
        stopFeedShooter();
    }

    /**
     * Runs shooter feed motor 
     */
    public void feedShooter(){
        feedShooter(-0.6);
        m_intakeMotor.set(0.15);
    }

    /**
     * Runs the shooter feed motor 
     * @param feedPercent speed to run it at
     */
    public void feedShooter(double feedPercent){
        m_intakeMotor.set(0.15);
        m_shooterFeedMotor.set(feedPercent);
    }

    /**
     * Stops shooter feed motor
     */
    public void stopFeedShooter(){
        m_shooterFeedMotor.set(0.0);
    }

    /**
     * Sets the intake motor's current limit
     * @param lim the current limit
     */
    public void setIntakeCurrentLimit(int lim){
        this.m_intakeMotor.setSmartCurrentLimit(lim);
        currentCurrentLimit = lim;
    }

    public void setIntakeToStuckCurrentLimit(){
        setIntakeCurrentLimit(INTAKE_ERROR_CURR_LIM);
    }

    public void setIntakeToUnStuckCurrentLimit(){
        setIntakeCurrentLimit(INTAKE_NORM_CURR_LIM);
    }

    public boolean getB1(){
        return b1;
    }

    public boolean getB2(){
        return b2;
    }

    public void setB1(boolean val){
        b1 = val;
    }

    /**
     * Gets the current value of the beam break underneath the shooter
     * @return whether the beam break is tripped or not
     */
    public boolean getShooterBeam(){
        return !(m_shooterBeamBreak.get());
    }

    public boolean getMidBeam(){
        return !(m_midFeedBeamBreak.get());
    }

    public boolean getLowBeam(){
        return !(m_lowFeedBeamBreak.get());
    }

    public boolean getHopperBeam(){
        
        return
                //(hopperLidar1.getRawDutyCycle() < hopperUpperBound && hopperLidar1.getRawDutyCycle() > hopperLowerBound)||
                (m_hopperLidar2.getRawDutyCycle() < hopperUpperBound && m_hopperLidar2.getRawDutyCycle() > hopperLowerBound);
                //(hopperLidar3.getRawDutyCycle() < hopperUpperBound && hopperLidar3.getRawDutyCycle() > 0.01);
        
    }

    public boolean getIntakeLidar(){
        return (m_hopperLidar1.getRawDutyCycle() > hopperLowerBound) &&
                (m_hopperLidar1.getRawDutyCycle() < hopperUpperBound);
    }
    
    public boolean getMidHopperLidar(){
        return (m_hopperLidar2.getRawDutyCycle() > hopperLowerBound) &&
                (m_hopperLidar2.getRawDutyCycle() < hopperUpperBound);
    }
    
    public boolean getBackHopperLidar(){
        return (m_hopperLidar3.getRawDutyCycle() > hopperLowerBound) &&
                (m_hopperLidar3.getRawDutyCycle() < hopperUpperBound);
    }

    public void toggleOverride(){
        override = !override;
    }

    public void toggleIntakePneumatic(){
        m_intakeSolenoid.toggle();
    }

    public void setReversed(boolean reverse){
        reversed = reverse;
    }

    public void setIntakeOnly(boolean intakeOnly){
        this.intakeOnly = intakeOnly;
    }
}
