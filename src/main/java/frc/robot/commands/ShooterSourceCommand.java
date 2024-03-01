// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterSourceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  
  public ShooterSourceCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooterSubsystem = shooter;
    m_intakeSubsystem = intake;
  
    addRequirements(shooter, intake);
  }

 
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_shooterSubsystem.setMotor1(-1 * Constants.ShooterConstants.kShooterSourceSpeed);
    m_shooterSubsystem.setMotor2(Constants.ShooterConstants.kShooterSourceSpeed);
    m_shooterSubsystem.setMotor3(-1 * Constants.ShooterConstants.kShooterSourceSpeed);
    m_intakeSubsystem.setspeed(0);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
