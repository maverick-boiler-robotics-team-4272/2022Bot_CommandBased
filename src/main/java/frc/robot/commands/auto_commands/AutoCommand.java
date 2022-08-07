package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutoCommand extends ParallelCommandGroup {
    private static final PIDController X_PID_CONTROLLER = new PIDController(0.2, 0.01, 0.0);
    private static final PIDController Y_PID_CONTROLLER = new PIDController(0.2, 0.01, 0.0);
    private static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(4.5, 0, 0,
    new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_SPEED, Drivetrain.MAX_ANGULAR_ACC));

    private static final HolonomicDriveController DRIVE_CONTROLLER = new HolonomicDriveController(X_PID_CONTROLLER, Y_PID_CONTROLLER, THETA_PID_CONTROLLER);

    private PathFollowCommand m_followCommand;
    protected Drivetrain m_drivetrain;

    protected AutoCommand(PathPlannerTrajectory path, Drivetrain drivetrain){
        m_followCommand = new PathFollowCommand(path, drivetrain::getRobotPose, drivetrain.getKinematics(), DRIVE_CONTROLLER, drivetrain::setSwerveModuleStates);
        m_drivetrain = drivetrain;

        m_drivetrain.setRobotPose(path.getInitialPose());

        addCommands(m_followCommand);
        addRequirements(drivetrain);
    }

    protected InstantCommand getStopCommand(){
        return new InstantCommand(m_followCommand::pauseTimer);
    }

    protected InstantCommand getStartCommand(){
        return new InstantCommand(m_followCommand::startTimer);
    }
}
