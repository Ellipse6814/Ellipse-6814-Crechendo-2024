// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterIntakeBottomRollerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intake;
  private boolean commandDone;
  
  public ShooterIntakeBottomRollerCommand(IntakeSubsystem intake) {
    m_intake = intake;

    addRequirements(intake);
  }

 
  @Override
  public void initialize() {
    commandDone = false;
  }


  @Override
  public void execute() {
    m_intake.setspeed(-1 * Constants.ShooterConstants.kShooterIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
