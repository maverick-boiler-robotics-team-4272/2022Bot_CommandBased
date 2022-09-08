// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.ShuffleboardTable;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double PI2 = Math.PI * 2;
    public static final double DEG_TO_RAD = Math.PI / 180.0;
    public static final double RAD_TO_DEG = 180.0 / Math.PI;

    public static final double VOLTAGE_COMPENSATION = 11.0;
    
    public static final class DrivetrainConstants {
        public static final double WHEEL_DISTANCE = Units.feetToMeters(1.0);

        public static final double MAX_LINEAR_SPEED = 4.4; // meters per second
        public static final double MAX_ANGULAR_SPEED = 4.0 * Math.PI;
        public static final double MAX_LINEAR_ACC = 4.75;
        public static final double MAX_ANGULAR_ACC = 1.5 * Math.PI;
    }

    public static final class ShooterConstants {
        public static final double HOOD_DEADZONE = 0.025;
        public static final double SHOOTER_DEADZONE = 100.0;

        public enum ShooterPositions{
            //shootAmt, hoodAmt, feedAmt
            FENDER_LOW(
                1500.0, -3.0, -0.75
            ),
            FENDER_HIGH(
                2100.0, -1.75, -0.7 //2250
            ),
            TARMAC(
                2220.0, -7.5, -0.9 //2300, -12.5
            ),
            LAUNCHPAD(
                2500.0, -11.0, -0.5
            ),
            EJECT(
                1000.0, 0.0, -0.5
            ),
            MID_TARMAC(
                2300.0, -10.0, -0.8
            ),
            MID_LAUNCHPAD(
                2450.0, -16.0, -0.8
            ),
            AUTO_TARMAC(
                2300.0, -15.25, -0.9
            );
    
            public final double shootAmt;
            public final double hoodAmt;
            public final double feedAmt;
    
            private ShooterPositions(double shootAmt, double hoodAmt, double feedAmt){
                this.shootAmt = shootAmt;
                this.hoodAmt = hoodAmt;
                this.feedAmt = feedAmt;
            }
    
        }
    }

    public static final class IntakeConstants {
        public static final int INTAKE_NORM_CURR_LIM = 55;
        public static final int INTAKE_ERROR_CURR_LIM = 80;
    }

    public static final class ClimberConstants {
        public static final float CLIMBER_SOFTLIMIT = -305.0f;
        public static final int CURRENT_LIMIT = 40;
    }

    public static final class Tables {
        public static final ShuffleboardTable TESTING_TABLE = ShuffleboardTable.getTable("Testing");
        public static final ShuffleboardTable SHOOTER_TABLE = ShuffleboardTable.getTable("Shooter");
        public static final ShuffleboardTable MODULE_TABLE = ShuffleboardTable.getTable("Modules");
        public static final ShuffleboardTable GAME_DATA_TABLE = ShuffleboardTable.getTable("Game Data");
    }
}
