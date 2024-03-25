// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmLockOnSpeakerCommand;
import frc.robot.commands.ArmRaiseCommand;
import frc.robot.commands.ShooterIntakeBottomRollerCommand;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.ShooterSourceCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveJoystickLockOnSpeakerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimbCommandJoystick;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();


  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

  private final Joystick joystick = new Joystick(2);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
      swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis) * 0.1,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis) * 0.1,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis) * 0.3,
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                m_armSubsystem));
    
    configureBindings();
    // Configure the trigger bindings
  }


  private void configureBindings() { 

    //new JoystickButton(joystick, 5).onTrue(new ArmRaiseCommand(m_armSubsystem, Math.toRadians(-1)).withTimeout(2.5));
    //new JoystickButton(joystick, 6).onTrue(new ArmRaiseCommand(m_armSubsystem, Math.toRadians(85)));
    //new JoystickButton(joystick, 1).onTrue(new ShooterSourceCommand(m_shooterSubsystem, m_intakeSubsystem));
    // new JoystickButton(joystick, 2).onTrue(new ShooterBackWheelCommand(m_shooterSubsystem).raceWith(new WaitCommand(1.0)).andThen(new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem)));
    
    //new JoystickButton(joystick, 4).onTrue(new ShooterIntakeCommand(m_shooterSubsystem, m_intakeSubsystem));
    //new JoystickButton(joystick, 1).onTrue(new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem));
    new JoystickButton(driverJoytick, 2).whileTrue(new ArmLockOnSpeakerCommand(m_armSubsystem, m_limelightSubsystem));
    new JoystickButton(driverJoytick, 3).whileTrue(new SwerveJoystickLockOnSpeakerCommand(
        swerveSubsystem, 
        () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis) * 0.1, 
        () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis) * 0.1, 
        () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        m_armSubsystem, 
        m_limelightSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
     // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        //pos x = forward
        //neg x = back
        //pos y = right
        //neg y = left
        //This is all you should need for the robot container implementation of the limelight stuff: Pose2d(limelightSubsystem.getBotposeTableEntry(0), limelightSubsystem.getBotposeTableEntry(1), limelightBotposeTableEntry(5))
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
                new Translation2d(2, 0)
                ),
        new Pose2d(4, 0, Rotation2d.fromDegrees(0)),
        trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new WaitCommand(8),
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}
