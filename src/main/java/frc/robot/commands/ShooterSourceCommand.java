// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterSourceCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooterSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;

  private boolean beamBreak;
  private boolean commandDone;
  
  public ShooterSourceCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
    m_shooterSubsystem = shooter;
    m_intakeSubsystem = intake;
  
    addRequirements(shooter, intake);
  }

 
  @Override
  public void initialize() {
    beamBreak = false;
    commandDone = false;
  }


  @Override
  public void execute() {
    SmartDashboard.putBoolean("beambreak1", m_shooterSubsystem.getSensor1());
    SmartDashboard.putBoolean("beambreak2", m_shooterSubsystem.getSensor2());
    //m_shooterSubsystem.setMotor1(-1 * 0.08);  ///NOTE FOR TOMORROW: motor 3 might be ok, but may need more, motor 2 needs more, keep motor 1
    m_shooterSubsystem.setMotor2(0.6);
    m_shooterSubsystem.setMotor3(-1 * 0.3);
    m_intakeSubsystem.setspeed(0);

    if(m_shooterSubsystem.getSensor1()){
      beamBreak = true;
    }

    //Speed up motor1 until first beambreak is hit
    if(beamBreak)
    {
      m_shooterSubsystem.setMotor1(-1 * 0.037);
    }
    else
    {
      m_shooterSubsystem.setMotor1(-1 * 0.2);
    }

    if(beamBreak && !m_shooterSubsystem.getSensor1()) {
      commandDone = true;
      
    }

    SmartDashboard.putBoolean("commandDone", commandDone);
  }
  
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
    m_intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return commandDone;
  }
}
