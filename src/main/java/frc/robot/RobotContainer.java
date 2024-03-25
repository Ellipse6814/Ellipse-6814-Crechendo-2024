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
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem(); 
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  private final SendableChooser<Command> autoChooser;

  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick joystick = new Joystick(2);
  
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {   
        NamedCommands.registerCommand("Intake Command", new ShooterIntakeCommand(m_shooterSubsystem, m_intakeSubsystem));
        NamedCommands.registerCommand("Shooter Command, Amp", new ShooterSpeakerAmpTrapCommand(m_shooterSubsystem, m_intakeSubsystem));

        configureBindings();

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                m_armSubsystem));
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
    // An example command will be run in autonomous
     // 1. Create trajectory settings
     /*    
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
        */
        /*
        return new SequentialCommandGroup(
                new WaitCommand(8),
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
        */
        
        return autoChooser.getSelected();
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Default Path");
        //return AutoBuilder.followPath(null);
  }
}
