// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ClimberCommands.ClimberRun;
import frc.robot.commands.IntakeCommands.IntakeRun;
import frc.robot.commands.IntakeCommands.IntakeSetup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.utils.JoystickAxes;
import frc.robot.utils.JoystickTrigger;
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
    Climber m_climber = new Climber();
    Intake m_intake = new Intake();
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
                            new ClimberRun(m_climber, operatorLeftStick::getDeadzonedY, operatorRightStick::getDeadzonedY)
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
        
        operatorRightTrigger.whenActive(
                                new IntakeSetup(m_intake, false)
                            ).whileActiveContinuous(
                                new IntakeRun(m_intake, operatorRightTrigger::getDeadzonedValue)
                            ).whenInactive(
                                m_intake::stopIntake, m_intake
                            );
        
        operatorLeftTrigger.and(operatorRightTrigger.negate())
                            .whenActive(
                                new IntakeSetup(m_intake, true)
                            ).whileActiveContinuous(
                                new IntakeRun(m_intake, operatorLeftTrigger::getDeadzonedValue)
                            ).whenInactive(
                                m_intake::stopIntake, m_intake
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
                        .whenActive(
                            new IntakeSetup(m_intake, false)
                        ).whileActiveContinuous(
                            new IntakeRun(m_intake, () -> {return 0.5;})
                        ).whenInactive(
                            m_intake::stopIntake, m_intake
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
}
