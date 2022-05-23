// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.utils.JoystickAxes;
import frc.robot.utils.JoystickTrigger;
import frc.robot.utils.Utils;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static frc.robot.utils.Utils.*;
import static frc.robot.Constants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's controllers are defined here..
  private XboxController m_driveController = new XboxController(0);

  private JoystickButton m_driveAButton = joystickButton(m_driveController, kA);
  private JoystickButton m_driveBButton = joystickButton(m_driveController, kB);
  private JoystickButton m_driveXButton = joystickButton(m_driveController, kX);
  private JoystickButton m_driveYButton = joystickButton(m_driveController, kY);

  private JoystickButton m_driveStartButton = joystickButton(m_driveController, kStart);
  private JoystickButton m_driveBackButton = joystickButton(m_driveController, kBack);

  private JoystickButton m_driveLeftBumper = joystickButton(m_driveController, kLeftBumper);
  private JoystickButton m_driveRightBumper = joystickButton(m_driveController, kRightBumper);

  private JoystickButton m_driveLeftStick = joystickButton(m_driveController, kLeftStick);
  private JoystickButton m_driveRightStick = joystickButton(m_driveController, kRightBumper);

  private JoystickTrigger m_driveLeftTrigger = joystickTrigger(m_driveController, kLeftTrigger);
  private JoystickTrigger m_driveRightTrigger = joystickTrigger(m_driveController, kRightTrigger);

  private JoystickAxes m_driveLeftJoystick = joystickAxes(m_driveController, kLeftX, kLeftY);
  private JoystickAxes m_driveRightJoystick = joystickAxes(m_driveController, kRightX, kRightY);


  private XboxController m_operatorController = new XboxController(1);

  private JoystickButton m_operatorAButton = joystickButton(m_operatorController, kA);
  private JoystickButton m_operatorBButton = joystickButton(m_operatorController, kB);
  private JoystickButton m_operatorXButton = joystickButton(m_operatorController, kX);
  private JoystickButton m_operatorYButton = joystickButton(m_operatorController, kY);

  private JoystickButton m_operatorStartButton = joystickButton(m_operatorController, kStart);
  private JoystickButton m_operatorBackButton = joystickButton(m_operatorController, kBack);

  private JoystickButton m_operatorLeftBumper = joystickButton(m_operatorController, kLeftBumper);
  private JoystickButton m_operatorRightBumper = joystickButton(m_operatorController, kRightBumper);

  private JoystickButton m_operatorLeftStick = joystickButton(m_operatorController, kLeftStick);
  private JoystickButton m_operatorRightStick = joystickButton(m_operatorController, kRightBumper);

  private JoystickTrigger m_operatorLeftTrigger = joystickTrigger(m_operatorController, kLeftTrigger);
  private JoystickTrigger m_operatorRightTrigger = joystickTrigger(m_operatorController, kRightTrigger);

  private JoystickAxes m_operatorLeftJoystick = joystickAxes(m_operatorController, kLeftX, kLeftY);
  private JoystickAxes m_operatorRightJoystick = joystickAxes(m_operatorController, kRightX, kRightY);
  // The robot's subsystems and commands are defined here...
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

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
