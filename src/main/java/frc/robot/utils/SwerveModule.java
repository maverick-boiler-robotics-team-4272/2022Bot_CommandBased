package frc.robot.utils;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.*;

public class SwerveModule {
    private static final ShuffleboardTable m_table = ShuffleboardTable.getTable("Modules");

    private static final double WHEEL_RADIUS = 2.0;
    private static final double DRIVE_RATIO = 6.75;
    private static final double STEER_RATIO = 150.0 / 7.0;
    private static final double MODULE_ROTATION_DEADZONE = 4.0;

    private static final double DRIVE_P = 0.003596;
    private static final double DRIVE_I = 0.0;
    private static final double DRIVE_D = 0.0;
    private static final double DRIVE_F = 0.47;

    private static final double STEER_P = 0.01;
    private static final double STEER_I = 0.0001;
    private static final double STEER_D = 0.0;
    private static final double STEER_F = 0.0;

    private CANSparkMax m_driveMotor;
    private RelativeEncoder m_driveEncoder;
    private SparkMaxPIDController m_drivePidController;
    private CANSparkMax m_rotationMotor;
    private RelativeEncoder m_rotationEncoder;
    private SparkMaxPIDController m_rotationPidController;
    private double m_offset;
    private CANCoder m_externalRotationEncoder;
    private final double m_flipped;
    private final int m_id;

    /**
     * 
     * @param moduleID - module id. 1 is front left, 2 is front right, 3 is back left, 4 is back right
     * @param offset - readout from encoder when the module is at 0
     * @param flipped - whether the encoder is upside-down
     */
    public SwerveModule(int moduleID, double offset, boolean flipped){
        m_driveMotor = new CANSparkMax(moduleID, MotorType.kBrushless);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_drivePidController = m_driveMotor.getPIDController();
        m_rotationMotor = new CANSparkMax(moduleID + 10, MotorType.kBrushless);
        m_rotationEncoder = m_rotationMotor.getEncoder();
        m_rotationPidController = m_rotationMotor.getPIDController();
        m_offset = offset;
        m_externalRotationEncoder = new CANCoder(moduleID + 20);
        m_flipped = flipped ? 1.0 : -1.0;

        m_drivePidController.setP(DRIVE_P);
        m_drivePidController.setI(DRIVE_I);
        m_drivePidController.setD(DRIVE_D);
        m_drivePidController.setFF(DRIVE_F);

        m_rotationPidController.setP(STEER_P);
        m_rotationPidController.setI(STEER_I);
        m_rotationPidController.setD(STEER_D);
        m_rotationPidController.setFF(STEER_F);

        m_driveMotor.setIdleMode(IdleMode.kBrake);
        m_rotationMotor.setIdleMode(IdleMode.kBrake);

        // m_driveMotor.burnFlash();
        // m_rotationMotor.burnFlash();
        m_id = moduleID;

        init();
    }
    private double getEncoderPosition(){
        return Utils.euclideanModulo((m_externalRotationEncoder.getAbsolutePosition() - m_offset) * m_flipped, 360.0);
    }

    /**
     * Initialization method for the swerve module
     */
    private void init(){
        //360.0 is the amount of degrees in a circle. This is useful, because
        //all our rotation math is done in degrees
        m_rotationEncoder.setPositionConversionFactor(360.0 / STEER_RATIO);
        
        //PI2 is 2 * pi, 60.0 is the amount of seconds in a minute
        m_driveEncoder.setVelocityConversionFactor(WHEEL_RADIUS * PI2 / (60.0 * DRIVE_RATIO * Units.metersToInches(1.0)));

        m_rotationEncoder.setPosition(getEncoderPosition());

        m_driveMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        m_rotationMotor.enableVoltageCompensation(VOLTAGE_COMPENSATION);
    }

    /**
     * 
     * @return offset of the module
     */
    public double getOffset() {
        return m_offset;
    }

    /**
     * 
     * @return angle that the module is at
     */
    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(m_rotationEncoder.getPosition());
    }

    /**
     * Sets the rotation and speed of the module. Automatically handles all
     * continuous math for direction to ensure the module never rotates more
     * than 90 degrees. Also ensures that if the module isn't driving anywhere,
     * don't rotate the module
     * @param desiredState desired state for the module
     */
    public void setState(SwerveModuleState desiredState){
        SwerveModuleState state = optimize(desiredState, getHeading());

        m_drivePidController.setReference(Units.metersToInches(state.speedMetersPerSecond)
        * 60.0 / (WHEEL_RADIUS * PI2) / DRIVE_RATIO, ControlType.kVelocity);

        if(state.speedMetersPerSecond != 0.0){
            m_rotationPidController.setReference(state.angle.getDegrees(), ControlType.kPosition);
        }
    }

    /**
     * 
     * @return Current state of the module
     */
    public SwerveModuleState getState(){
        Rotation2d heading = getHeading();

        return new SwerveModuleState(-m_driveEncoder.getVelocity(), new Rotation2d(-heading.getRadians()));
    }

    public void updateRotation(){
        double encoderPosition = getEncoderPosition();
        double rotationPosition = Utils.euclideanModulo(m_rotationEncoder.getPosition(), 360.0);

        m_table.putNumber("Encoder Position " + m_id, encoderPosition);
        m_table.putNumber("Rotation Position" + m_id, rotationPosition);

        if(Math.abs(rotationPosition - encoderPosition) > MODULE_ROTATION_DEADZONE){
            m_rotationEncoder.setPosition(getEncoderPosition());
            m_table.putBoolean("Updated " + m_id, true);
        } else {
            m_table.putBoolean("Updated " + m_id, false);
        }
    }

    /**
     * Optimize function that automatically handles continuous math for setting the state
     * @param desiredState where you want the module to be
     * @param currentAngle what direction the module is facing
     * @return optimized module state
     */
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
