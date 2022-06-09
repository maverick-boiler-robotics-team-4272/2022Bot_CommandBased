package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private static final double WHEEL_DISTANCE = Units.feetToMeters(1.0);

    private final Pigeon2 m_pigeon = new Pigeon2(25);

    /**
     * X+ - right
     * Y+ - forward
     */
    private final Translation2d m_frontRightPosition = new Translation2d(WHEEL_DISTANCE, WHEEL_DISTANCE);
    private final Translation2d m_frontLeftPosition = new Translation2d(-WHEEL_DISTANCE, WHEEL_DISTANCE);
    private final Translation2d m_backRightPosition = new Translation2d( WHEEL_DISTANCE,-WHEEL_DISTANCE);
    private final Translation2d m_backLeftPosition = new Translation2d( -WHEEL_DISTANCE,-WHEEL_DISTANCE);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontRightPosition, m_frontLeftPosition, m_backRightPosition, m_backLeftPosition);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(m_pigeon.getYaw()));

    private final SwerveModule m_frontRightModule = new SwerveModule(2, 0.0, false);
    private final SwerveModule m_frontLeftModule  = new SwerveModule(1, 0.0, false);
    private final SwerveModule m_backRightModule  = new SwerveModule(4, 0.0, false);
    private final SwerveModule m_backLeftModule   = new SwerveModule(3, 0.0, false);

    public Drivetrain(){
        
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
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
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
}