package frc.robot.utils;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class SwerveModule {
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_rotationMotor;
    private RelativeEncoder m_rotationEncoder;
    private double m_offset;
    private CANCoder m_externalRotationEncoder;

    public SwerveModule(CANSparkMax driveMotor, CANSparkMax rotationMotor, double offset, CANCoder rotationEncoder){
        m_driveMotor = driveMotor;
        m_rotationMotor = rotationMotor;
        m_rotationEncoder = m_rotationMotor.getEncoder();
        m_offset = offset;
        m_externalRotationEncoder = rotationEncoder;
    }
}
