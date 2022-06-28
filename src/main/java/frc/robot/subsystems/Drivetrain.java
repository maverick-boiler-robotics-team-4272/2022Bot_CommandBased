package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private static final double WHEEL_DISTANCE = Units.feetToMeters(1.0);

    public static final double MAX_LINEAR_SPEED = 4.4; // meters per second
    public static final double MAX_ANGULAR_SPEED = 8.0 * Math.PI;

    private final Pigeon2 m_pigeon = new Pigeon2(25);

    /**
     * X+ - right
     * Y+ - forward
     */
    private final Translation2d m_frontLeftPosition = new Translation2d(-WHEEL_DISTANCE, WHEEL_DISTANCE);
    private final Translation2d m_frontRightPosition = new Translation2d(WHEEL_DISTANCE, WHEEL_DISTANCE);
    private final Translation2d m_backLeftPosition = new Translation2d( -WHEEL_DISTANCE,-WHEEL_DISTANCE);
    private final Translation2d m_backRightPosition = new Translation2d( WHEEL_DISTANCE,-WHEEL_DISTANCE);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftPosition, m_backLeftPosition, m_frontRightPosition, m_backRightPosition);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(m_pigeon.getYaw()));

    private final SwerveModule m_frontLeftModule  = new SwerveModule(1, 311.0,  true);
    private final SwerveModule m_frontRightModule = new SwerveModule(2, 182.0, false);
    private final SwerveModule m_backLeftModule   = new SwerveModule(3, 209.0, false);
    private final SwerveModule m_backRightModule  = new SwerveModule(4,  59.0, false);

    private boolean m_fieldRelative = true;

    public Drivetrain(){
        zeroPigeon();
    }
    
    /**
     * 
     * @param degrees - Heading for robot in degrees. Angle should be CCW+
     */
    public void setPigeonHeading(double degrees){
        m_pigeon.setYaw(degrees);
    }

    /**
     * 
     * @param heading - Heading for robot as a Rotation2d. Angle should be CCW+
     */
    public void setPigeonHeading(Rotation2d heading){
        setPigeonHeading(heading.getDegrees());
    }

    /**
     * Zeros what the pigeon's current heading is.
     * <p>Essentially says that the current direction of the robot is the new forward
     */
    public void zeroPigeon(){
        setPigeonHeading(0.0);
    }

    public Rotation2d getPigeonHeading(){
        return Rotation2d.fromDegrees(-m_pigeon.getYaw());
    }

    public Pose2d getRobotPose(){
        return m_odometry.getPoseMeters();
    }

    public void setRobotPose(Pose2d pose){
        m_odometry.resetPosition(pose, getPigeonHeading());
    }

    /**
     * Resets the robot pose to be at (0, 0) with 0 rotation
     */
    public void resetRobotPose(){
        setRobotPose(new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
    }

    public void setSwerveModuleStates(SwerveModuleState... states){
        m_frontLeftModule.setState(states[0]);
        m_frontRightModule.setState(states[1]);
        m_backLeftModule.setState(states[2]);
        m_backRightModule.setState(states[3]);
    }

    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] states = {
            m_frontLeftModule.getState(),
            m_frontRightModule.getState(),
            m_backLeftModule.getState(),
            m_backRightModule.getState()
        };
        return states;
    }

    public void toggleFieldRelative(){
        m_fieldRelative = !m_fieldRelative;
    }

    public boolean getFieldRelative(){
        return m_fieldRelative;
    }

    public void drive(double xSpeed, double ySpeed, double rotationSpeed){
        ChassisSpeeds speeds = m_fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, rotationSpeed, getPigeonHeading())
                                               : new ChassisSpeeds(ySpeed, -xSpeed, rotationSpeed);
        
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
        setSwerveModuleStates(states);
        updateOdometry();
    }

    public void updateOdometry(){
        m_odometry.update(getPigeonHeading(), getSwerveModuleStates());
    }
}