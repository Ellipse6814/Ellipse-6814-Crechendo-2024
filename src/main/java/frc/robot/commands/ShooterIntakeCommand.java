// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intake;
  private boolean beamBreak;
  private boolean commandDone;
  
  public ShooterIntakeCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooterSubsystem = shooter;
    
    m_intake = intake;

    addRequirements(shooter, intake);
  }

 
  @Override
  public void initialize() {
  }


  @Override
  public void execute() {
      SmartDashboard.putString("is it running?", "yes");
      m_shooterSubsystem.setLeftVortex(0);
      m_shooterSubsystem.setRightVortex(0);
      m_shooterSubsystem.setMotor3(0);
      m_shooterSubsystem.setMotor4(ShooterConstants.kShooterIntakeSpeed - 0.2);
      m_intake.setspeed(-1 * Constants.ShooterConstants.kShooterIntakeSpeed);
    


  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_intake.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooterSubsystem.getSensor1();
  }
}
