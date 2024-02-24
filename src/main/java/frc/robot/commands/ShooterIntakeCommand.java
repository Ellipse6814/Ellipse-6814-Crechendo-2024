// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intake;

  
  public ShooterIntakeCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooterSubsystem = shooter;
    
    m_intake = intake;

    addRequirements(shooter, intake);
  }

 
  @Override
  public void initialize() {
    m_shooterSubsystem.setMotor1(-1 * Constants.ShooterConstants.kShooterIntakeSpeed);
    m_shooterSubsystem.setMotor2(0);
    m_shooterSubsystem.setMotor3(Constants.ShooterConstants.kShooterIntakeSpeed);
    m_intake.setspeed(-1 * Constants.ShooterConstants.kShooterIntakeSpeed);

  }


  @Override
  public void execute() {}

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
