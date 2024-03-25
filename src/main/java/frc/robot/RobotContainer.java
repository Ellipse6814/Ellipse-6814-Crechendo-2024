// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmRaiseCommand;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.ShooterSpeakerAmpTrapCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommandJoystick;
import frc.robot.commands.LimelightUpdateCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here...
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem(); 
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  
  private final SendableChooser<Command> autoChooser;
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {   
        NamedCommands.registerCommand("Intake Command", new ShooterIntakeCommand(m_shooterSubsystem, m_intakeSubsystem));
        NamedCommands.registerCommand("Shooter Command, Amp", new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem));

        configureBindings();

  private final Joystick joystick = new Joystick(0);

  private final XboxController driverJoystick = new XboxController(5);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
        m_limelightSubsystem.setDefaultCommand(new LimelightUpdateCommand(m_limelightSubsystem, swerveSubsystem));
    
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                m_armSubsystem));
    
    autoChooser = AutoBuilder.buildAutoChooser();
        
    SmartDashboard.putData("Auto Chooser", autoChooser);
        
    m_climbSubsystem.setDefaultCommand(new ClimbCommandJoystick(m_climbSubsystem, () -> joystick.getRawAxis(1), () -> joystick.getRawAxis(5)));
    configureBindings();
  }

  private void configureBindings() { 
    new JoystickButton(joystick, 5).onTrue(new ArmRaiseCommand(m_armSubsystem, Math.toRadians(-1)).withTimeout(2.5));
    new JoystickButton(joystick, 6).onTrue(new ArmRaiseCommand(m_armSubsystem, Math.toRadians(85)));    
    
    new JoystickButton(joystick, 4).onTrue(new ShooterIntakeCommand(m_shooterSubsystem, m_intakeSubsystem));
    new JoystickButton(joystick, 1).onTrue(new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Default Path");
        //return AutoBuilder.followPath(null);
  }
}
