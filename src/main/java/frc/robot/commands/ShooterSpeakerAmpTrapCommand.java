// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** An example command that uses an example subsystem. */
public class ShooterSpeakerAmpTrapCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private final IntakeSubsystem m_intake;

  
  public ShooterSpeakerAmpTrapCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    m_shooter = shooterSubsystem;
    m_intake = intakeSubsystem;
  
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

 
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_shooter.setMotor1(ShooterConstants.kShooterSpeakerAmpSpeed);
    m_shooter.setMotor2(ShooterConstants.kShooterSpeakerAmpSpeed);
    m_shooter.setMotor3(ShooterConstants.kShooterSpeakerAmpSpeed);
    m_intake.setspeed(0);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
