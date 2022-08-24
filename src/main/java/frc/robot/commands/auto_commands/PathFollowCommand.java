//Changed Slightly from the Path planner swerve follow command to allow pausing and restarting the timer
package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PathFollowCommand extends CommandBase{


/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link PathPlannerTrajectory}
 * with a swerve drive.
 *
 * <p>
 * This command outputs the raw desired Swerve Module States
 * ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from
 * those and used in
 * velocity PIDs.
 *
 * <p>
 * The robot angle controller does not follow the angle given by the trajectory
 * but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
    private final Timer m_timer = new Timer();
    private boolean m_timerPaused = true;
    private boolean m_stopped = false;
    private final PathPlannerTrajectory m_trajectory;
    private final PIDController m_xPidController;
    private final PIDController m_yPidController;
    private final ProfiledPIDController m_thetaPidController;
    private final Drivetrain m_drivetrain;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to
     *                           provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param thetaController    The Trajectory Tracker PID controller for angle for
     *                           the robot.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public PathFollowCommand(
            PathPlannerTrajectory trajectory,
            PIDController xPid,
            PIDController yPid,
            ProfiledPIDController thetaPid,
            Drivetrain drivetrain) {
        m_trajectory = trajectory;
        m_drivetrain = drivetrain;
        m_xPidController = xPid;
        m_yPidController = yPid;
        m_thetaPidController = thetaPid;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setRobotPose(m_trajectory.getInitialPose());
        m_drivetrain.setPigeonHeading(m_trajectory.getInitialPose().getRotation());

        m_timer.reset();
        m_timer.start();
        m_timerPaused = false;
        m_stopped = false;
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        if(m_timerPaused) {
            if(!m_stopped){
                m_stopped = true;
                m_drivetrain.drive(0.0, 0.0, 0.0);
            }
            return;
        }

        if(m_stopped) m_stopped = false;
        double curTime = m_timer.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        Pose2d desiredPose = new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation);
        Pose2d currentPose = m_drivetrain.getRobotPose();

        double xSpeed = m_xPidController.calculate(currentPose.getX(), desiredPose.getX());
        double ySpeed = m_yPidController.calculate(-currentPose.getY(), -desiredPose.getY());
        double thetaSpeed = m_thetaPidController.calculate(currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());

        m_drivetrain.drive(ySpeed, xSpeed, thetaSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timerPaused = true;
        m_drivetrain.drive(0.0, 0.0, 0.0);
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }

    public void pauseTimer(){
        m_timer.stop();
        m_timerPaused = true;
    }

    public void startTimer(){
        m_timer.start();
        m_timerPaused = false;
    }
}
