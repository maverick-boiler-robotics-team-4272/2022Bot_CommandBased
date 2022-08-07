//Changed Slightly from the Path planner swerve follow command to allow pausing and restarting the timer
package frc.robot.commands.auto_commands;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

@SuppressWarnings("MemberName")
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
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;

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
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            HolonomicDriveController controller,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = controller;

        m_outputModuleStates = outputModuleStates;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_timerPaused = false;
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        if(m_timerPaused){
            SwerveModuleState[] states = {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
            };
            
            m_outputModuleStates.accept(states);
            return;
        }
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timerPaused = true;
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
