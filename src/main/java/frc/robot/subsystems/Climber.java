package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

    private CANSparkMax m_leftMotor = new CANSparkMax(17, MotorType.kBrushless);
    private CANSparkMax m_rightMotor = new CANSparkMax(7, MotorType.kBrushless);

    private DoubleSolenoid m_solenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 3, 2);

    public Climber(){
        m_solenoid.set(Value.kReverse);

        m_leftMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        m_rightMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);

        m_leftMotor.setSmartCurrentLimit(CURRENT_LIMIT);
        m_rightMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        m_leftMotor.burnFlash();
        m_rightMotor.burnFlash();
    }

    public void runClimbers(double leftSpeed, double rightSpeed){
        m_leftMotor.set(leftSpeed);
        m_rightMotor.set(rightSpeed);
    }

    public void removeSoftLimits(){
        m_leftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_rightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);

        m_leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    public void reapplySoftLimits(){
        m_leftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_rightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);

        m_leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_leftMotor.setSoftLimit(SoftLimitDirection.kReverse, CLIMBER_SOFTLIMIT);
        m_rightMotor.setSoftLimit(SoftLimitDirection.kReverse, CLIMBER_SOFTLIMIT);

        m_leftMotor.getEncoder().setPosition(0.0);
        m_rightMotor.getEncoder().setPosition(0.0);
    }

    public void togglePneumatics(){
        m_solenoid.toggle();
    }
}