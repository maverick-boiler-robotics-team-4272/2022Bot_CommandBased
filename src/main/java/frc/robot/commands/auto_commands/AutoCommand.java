package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoCommand extends ParallelCommandGroup {
    private static final PIDController X_PID_CONTROLLER = new PIDController(0.05, 0.0, 0.0);
    private static final PIDController Y_PID_CONTROLLER = new PIDController(0.05, 0.0, 0.0);
    private static final ProfiledPIDController THETA_PID_CONTROLLER = new ProfiledPIDController(0.25, 0, 0,
    new TrapezoidProfile.Constraints(Drivetrain.MAX_ANGULAR_SPEED, Drivetrain.MAX_ANGULAR_ACC));

    private PathFollowCommand m_followCommand;
    protected Shooter m_shooter;
    protected Intake m_intake;
    protected Drivetrain m_drivetrain;

    protected AutoCommand(PathPlannerTrajectory path, Shooter shooter, Intake intake, Drivetrain drivetrain){
        m_followCommand = new PathFollowCommand(path, X_PID_CONTROLLER, Y_PID_CONTROLLER, THETA_PID_CONTROLLER, drivetrain);
        m_shooter = shooter;
        m_intake = intake;
        m_drivetrain = drivetrain;

        addCommands(m_followCommand);
        addRequirements(drivetrain);
    }

    protected AutoCommand(String name, Shooter shooter, Intake intake, Drivetrain drivetrain){
        this(
            PathPlanner.loadPath(name, Drivetrain.MAX_LINEAR_SPEED, Drivetrain.MAX_LINEAR_ACC),
            shooter,
            intake,
            drivetrain
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0.0, 0.0, 0.0);
        m_shooter.stopShooter();
        m_intake.stopFeedShooter();
        m_intake.stopIntake();
    }

    

    protected InstantCommand getStopCommand(){
        return new InstantCommand(m_followCommand::pauseTimer);
    }

    protected InstantCommand getStartCommand(){
        return new InstantCommand(m_followCommand::startTimer);
    }

    protected PathFollowCommand getFollowCommand(){
        return m_followCommand;
    }


    public static interface AutoCommandCreator {
        public AutoCommand create(Shooter shooter, Intake intake, Drivetrain drivetrain);
    }

    public enum Paths {
        TERMINAL_2_BALL(Terminal2Ball::new),
        TUNE_PATH(TuneAutoCommand::new),
        SHOOT_N_BACKUP(BackShootCommand::new);

        public final AutoCommandCreator m_creator;
        private Paths(AutoCommandCreator creator){
            m_creator = creator;
        }
    }
}
