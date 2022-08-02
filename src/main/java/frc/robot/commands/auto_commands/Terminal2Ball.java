package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.commands.auto_commands.AutoCommandGlobals.*;

public class Terminal2Ball extends ParallelCommandGroup {
    public Terminal2Ball(Drivetrain drivetrain){
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath", Drivetrain.MAX_LINEAR_ACC, Drivetrain.MAX_ANGULAR_ACC);
        PathFollowCommand followCommand = new PathFollowCommand(trajectory, drivetrain::getRobotPose, drivetrain.getKinematics(), X_PID_CONTROLLER, Y_PID_CONTROLLER, THETA_PID_CONTROLLER, drivetrain::setSwerveModuleStates, drivetrain);

        drivetrain.setRobotPose(trajectory.sample(0.0).poseMeters);

        addCommands(
            followCommand,
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new InstantCommand(followCommand::pauseTimer),
                new WaitCommand(1),
                new InstantCommand(followCommand::startTimer)
            )
        );
    }
}
