// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmRaiseCommand;
import frc.robot.commands.ShooterBackWheelCommand;
import frc.robot.commands.ShooterIntakeBottomRollerCommand;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.ShooterSourceCommand;
import frc.robot.commands.ShooterSpeakerAmpTrapCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommandJoystick;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem(); 
  private final IntakeSubsystem m_exampleSubsystem = new IntakeSubsystem();

  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

  private final Joystick joystick = new Joystick(0);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_ClimbSubsystem.setDefaultCommand(new ClimbCommandJoystick(m_ClimbSubsystem, () -> driverJoytick.getRawAxis(2) * 0.3, () -> driverJoytick.getRawAxis(4) * 0.3));
    configureBindings();
  }


  private void configureBindings() {
    // new JoystickButton(joystick, 1).onTrue(new ShooterSourceCommand(m_shooterSubsystem, m_intakeSubsystem));
    // new JoystickButton(joystick, 2).onTrue(new ShooterBackWheelCommand(m_shooterSubsystem).raceWith(new WaitCommand(1.0)).andThen(new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem)));
    new JoystickButton(joystick, 3).onTrue(new ArmRaiseCommand(m_armSubsystem, Math.toRadians(60)));
    //new JoystickButton(joystick, 4).onTrue(new ShooterIntakeCommand(m_shooterSubsystem, m_intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   return null;
  }
}
