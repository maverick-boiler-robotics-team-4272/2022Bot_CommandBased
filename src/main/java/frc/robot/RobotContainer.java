// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AimShootCommand;
import frc.robot.commands.ClimberRunCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FixHoodCommand;
import frc.robot.commands.IntakeRunCommand;
import frc.robot.commands.SetHoodCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.JoystickAxes;
import frc.robot.utils.JoystickTrigger;
import frc.robot.utils.Limelight;
import frc.robot.utils.Utils;
import frc.robot.utils.XBoxController;

import frc.robot.utils.JoystickAxes.DeadzoneMode;
import frc.robot.utils.XBoxController.Axes;
import frc.robot.utils.XBoxController.Buttons;
import frc.robot.utils.XBoxController.Triggers;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's controllers are defined here...
    private XBoxController m_driveController = new XBoxController(0);
    private XBoxController m_operatorController = new XBoxController(1);
    
    // The robot's subsystems and commands are defined here...
    private Climber m_climber = new Climber();
    private Drivetrain m_drivetrain = new Drivetrain();
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter(m_intake);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Limelight.setLEDMode(Limelight.LEDMode.OFF);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        JoystickAxes operatorLeftStick = m_operatorController.getAxis(Axes.LEFT_STICK);
        JoystickAxes operatorRightStick = m_operatorController.getAxis(Axes.RIGHT_STICK);

        JoystickTrigger operatorRightTrigger = m_operatorController.getTrigger(Triggers.RIGHT_TRIGGER);
        JoystickTrigger operatorLeftTrigger = m_operatorController.getTrigger(Triggers.LEFT_TRIGGER);

        operatorLeftStick.setMode(DeadzoneMode.Y_AXIS);
        operatorRightStick.setMode(DeadzoneMode.Y_AXIS);
        operatorLeftStick.or(operatorRightStick)
                        .whileActiveContinuous(
                            new ClimberRunCommand(m_climber, operatorLeftStick::getDeadzonedY, operatorRightStick::getDeadzonedY)
                        );
        
        m_operatorController.getButton(Buttons.Y_BUTTON)
                            .whenPressed(
                                m_climber::togglePneumatics, m_climber
                            );
        
        m_operatorController.getButton(Buttons.LEFT_BUMPER)
                            .whenPressed(
                                m_climber::removeSoftLimits, m_climber
                            ).whenReleased(
                                m_climber::reapplySoftLimits, m_climber
                            );
        
        operatorRightTrigger.whileActiveContinuous(
                                new IntakeRunCommand(m_intake, false, operatorRightTrigger::getDeadzonedValue)
                            );
        
        operatorLeftTrigger.and(operatorRightTrigger.negate())
                            .whileActiveContinuous(
                                new IntakeRunCommand(m_intake, true, operatorLeftTrigger::getDeadzonedValue)
                            );

        
        JoystickAxes driveLeftStick = m_driveController.getAxis(Axes.LEFT_STICK);
        JoystickAxes driveRightStick = m_driveController.getAxis(Axes.RIGHT_STICK);

        driveLeftStick.setMode(DeadzoneMode.MAGNITUDE);
        driveRightStick.setMode(DeadzoneMode.X_AXIS);

        driveLeftStick.or(driveRightStick)
                    .whileActiveContinuous(
                        new DriveCommand(m_drivetrain, driveLeftStick::getDeadzonedX, driveLeftStick::getDeadzonedY, driveRightStick::getDeadzonedX)
                    );
        
        m_driveController.getButton(Buttons.START_BUTTON)
                        .whenPressed(
                            new InstantCommand(m_drivetrain::toggleFieldRelative, m_drivetrain)
                        );
                         
        m_driveController.getButton(Buttons.B_BUTTON)
                        .whenPressed(
                            new InstantCommand(m_drivetrain::zeroPigeon, m_drivetrain)
                        );


        m_operatorController.getButton(Buttons.A_BUTTON)
                            .whenActive(
                                m_intake::toggleOverride, m_intake
                            );
        
        m_driveController.getButton(Buttons.A_BUTTON)
                        .whenPressed(
                            m_intake::toggleIntakePneumatic, m_intake
                        );

        m_driveController.getButton(Buttons.RIGHT_BUMPER)
                        .and(operatorRightTrigger.negate())
                        .and(operatorLeftTrigger.negate())
                        .whileActiveContinuous(
                            new IntakeRunCommand(m_intake, false, () -> {
                                return 0.5;
                            })
                        );

        m_driveController.getTrigger(Triggers.LEFT_TRIGGER)
                        .whileActiveContinuous(
                            new AimShootCommand(m_shooter, m_intake, m_drivetrain, new PIDController(0.01, 0.0, 0.0))
                        );

        m_driveController.getTrigger(Triggers.RIGHT_TRIGGER)
                        .and(m_driveController.getPOV().negate())
                        .whileActiveContinuous(
                            new ShootCommand(m_shooter, m_intake)
                        );

        m_driveController.getPOV()
                        .whenActive(
                            new SetHoodCommand(m_shooter, m_driveController), false
                        );

        m_driveController.getButton(Buttons.BACK_BUTTON)
                        .whenActive(
                            new FixHoodCommand(m_shooter), false
                        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new InstantCommand(Utils::noop);
    }
    
    public Climber getClimber(){
        return m_climber;
    }

    public Drivetrain getDrivetrain(){
        return m_drivetrain;
    }

    public Intake getIntake(){
        return m_intake;
    }
}
