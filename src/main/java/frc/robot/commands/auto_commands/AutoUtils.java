package frc.robot.commands.auto_commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.DrivetrainConstants.*;

public class AutoUtils {
    private AutoUtils() {
        throw new UnsupportedOperationException("Cannot construct an AutoUtils instance");
    }

    public static PathPlannerTrajectory loadPath(String name){
        return PathPlanner.loadPath(name, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
    }

    public static interface AutoCreator {
        CommandGroupBase create(Shooter shooter, Intake intake, Drivetrain drivetrain);
    }

    public enum Paths {
        BACK_N_SHOOT(BackShootCommand::new),
        TUNE_PATH(TuneAutoCommand::new);

        public final AutoCreator creator;
        private Paths(AutoCreator creator){
            this.creator = creator;
        }
    }
}
