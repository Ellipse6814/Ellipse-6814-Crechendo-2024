// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_subsystem;

  private final double leftSetpoint, rightSetpoint;
  private final PIDController pidController = new PIDController(ClimbConstants.kp, ClimbConstants.ki, ClimbConstants.kd);
  
  public ClimbCommand(ClimbSubsystem subsystem, double leftSetpoint, double rightSetpoint) {
    m_subsystem = subsystem;
    this.leftSetpoint = leftSetpoint;
    this.rightSetpoint = rightSetpoint;
    addRequirements(subsystem);
  }

 
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    double leftError = pidController.calculate(m_subsystem.getLeftEncoder() * ClimbConstants.encoderTicks2Meters, leftSetpoint);
    double rightError = pidController.calculate(m_subsystem.getRightEncoder() * ClimbConstants.encoderTicks2Meters, leftSetpoint);

    m_subsystem.setLeftMotor(leftError);
    m_subsystem.setRightMotor(rightError);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
