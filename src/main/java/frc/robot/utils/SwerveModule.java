package frc.robot.utils;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class SwerveModule {
    private static final double WHEEL_RADIUS = 2.0;
    private static final double DRIVE_RATIO = 6.75;
    private static final double STEER_RATIO = 150.0 / 7.0;

    private CANSparkMax m_driveMotor;
    private RelativeEncoder m_driveEncoder;
    private SparkMaxPIDController m_drivePidController;
    private CANSparkMax m_rotationMotor;
    private RelativeEncoder m_rotationEncoder;
    private SparkMaxPIDController m_rotationPidController;
    private double m_offset;
    private CANCoder m_externalRotationEncoder;
    private double m_flipped;

    public SwerveModule(int moduleID, double offset, boolean flipped){
        m_driveMotor = new CANSparkMax(moduleID, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_drivePidController = m_driveMotor.getPIDController();
        m_rotationMotor = new CANSparkMax(moduleID + 10, MotorType.kBrushless);
        m_rotationEncoder = m_rotationMotor.getEncoder();
        m_rotationPidController = m_rotationMotor.getPIDController();
        m_offset = offset;
        m_externalRotationEncoder = new CANCoder(moduleID + 20);
        m_flipped = flipped ? -1.0 : 1.0;
        init();
    }

    private void init(){
        //360.0 is the amount of degrees in a circle. Useful, because
        //all our rotation math is done in degrees
        m_rotationEncoder.setPositionConversionFactor(360.0 / STEER_RATIO);
        
        //PI2 is 2 * pi, 60.0 is the amount of seconds in a minute
        m_driveEncoder.setVelocityConversionFactor(WHEEL_RADIUS * PI2 / (60.0 * DRIVE_RATIO * Units.metersToInches(1.0)));

        double current = (m_externalRotationEncoder.getAbsolutePosition() - m_offset) % 360.0;
        m_rotationEncoder.setPosition(m_flipped * current);
    }

    public double getOffset() {
        return m_offset;
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(m_rotationEncoder.getPosition());
    }

    public void setState(SwerveModuleState desiredState){
        SwerveModuleState state = optimize(desiredState, getHeading());

        m_drivePidController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        if(state.speedMetersPerSecond != 0.0){
            m_rotationPidController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        }
    }

    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        boolean inverted = false;

        double desiredDegrees = desiredState.angle.getDegrees() % 360.0;
        if(desiredDegrees < 0.0){
            desiredDegrees += 360.0;
        }

        double currentDegrees = currentAngle.getDegrees();
        double currentMod = currentDegrees % 360.0;
        if(currentMod < 0.0){
            currentMod += 360.0;
        }

        if(Math.abs(currentMod - desiredDegrees) > 90.0 && Math.abs(currentMod - desiredDegrees) <= 270.0){
            inverted = true;
            desiredDegrees -= 180.0;
        }

        double deltaAngle = desiredDegrees - currentMod;
        if(deltaAngle < 0.0){
            deltaAngle += 360.0;
        }

        double counterClockWiseAngle = deltaAngle;
        double clockWiseAngle = deltaAngle - 360.0;

        if(Math.abs(counterClockWiseAngle) < Math.abs(clockWiseAngle)){
            desiredDegrees = counterClockWiseAngle;
        }else{
            desiredDegrees = clockWiseAngle;
        }

        double magnitude = desiredState.speedMetersPerSecond;

        if(inverted){
            magnitude *= -1.0;
        }

        desiredDegrees += currentDegrees;

        return new SwerveModuleState(magnitude, Rotation2d.fromDegrees(desiredDegrees));
    }
}
