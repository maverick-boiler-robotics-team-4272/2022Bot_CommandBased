//Changed Slightly from the Path planner swerve follow command to allow pausing and restarting the timer
package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PathFollowCommand extends CommandBase {

    public static final PIDController DEFAULT_X_PID = new PIDController(0.6, 0.0, 0.0);
    public static final PIDController DEFAULT_Y_PID = new PIDController(0.6, 0.0, 0.0);
    // private static final ProfiledPIDController DEFAULT_THETA_PID = new ProfiledPIDController(
    //     0.05, 0.0, 0.0, new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_SPEED, Drivetrain.MAX_ANGULAR_ACC)
    // );
    public static final PIDController DEFAULT_THETA_PID = new PIDController(0.6, 0.0, 0.0);

    private static boolean initted = false;

    private final Timer m_timer = new Timer();
    private boolean m_timerPaused = true;
    private final PathPlannerTrajectory m_trajectory;
    private final PIDController m_xPidController;
    private final PIDController m_yPidController;
    // private final ProfiledPIDController m_thetaPidController;
    private final PIDController m_thetaPidController;
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
            // ProfiledPIDController thetaPid,
            PIDController thetaPid,
            Drivetrain drivetrain) {

        if(!initted) {
            DEFAULT_THETA_PID.enableContinuousInput(-Math.PI, Math.PI);
            initted = true;
        }
        
        m_trajectory = trajectory;
        m_drivetrain = drivetrain;
        m_xPidController = xPid;
        m_yPidController = yPid;
        m_thetaPidController = thetaPid;

        addRequirements(drivetrain);
    }

    public PathFollowCommand(
        PathPlannerTrajectory trajectory,
        Drivetrain drivetrain) {
            this(
                trajectory, DEFAULT_X_PID, DEFAULT_Y_PID, DEFAULT_THETA_PID, drivetrain    
            );
        }

    @Override
    public void initialize() {
        PathPlannerState initialState = (PathPlannerState) m_trajectory.sample(0.0);

        m_drivetrain.setPigeonHeading(0.0);
        m_drivetrain.setRobotPose(AutoUtils.poseFromPathPlannerState(initialState));
                
        m_drivetrain.setPigeonHeading(initialState.holonomicRotation);

        m_timer.reset();
        m_timer.start();
        m_timerPaused = false;
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        if(m_timerPaused) {
            m_drivetrain.drive(0.0, 0.0, 0.0);

            return;
        }

        double curTime = m_timer.get();
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        Pose2d desiredPose = AutoUtils.poseFromPathPlannerState(desiredState);
        Pose2d currentPose = m_drivetrain.getRobotPose();

        double xSpeed = m_xPidController.calculate(currentPose.getX(), desiredPose.getX());
        double ySpeed = m_yPidController.calculate(currentPose.getY(), desiredPose.getY());
        double thetaSpeed = m_thetaPidController.calculate(m_drivetrain.getPigeonHeading().getRadians(), desiredPose.getRotation().getRadians());

        m_drivetrain.driveFieldCoords(-xSpeed, ySpeed, thetaSpeed);
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
