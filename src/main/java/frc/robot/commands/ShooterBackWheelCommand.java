// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterBackWheelCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;

  
  public ShooterBackWheelCommand(ShooterSubsystem shooterSubsystem) {
    m_shooter = shooterSubsystem;
  
    addRequirements(shooterSubsystem);
  }

 
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_shooter.setMotor2(ShooterConstants.kShooterSpeakerAmpSpeed);
    //m_shooter.setMotor3(ShooterConstants.kShooterSpeakerAmpSpeed);
  }

  
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
