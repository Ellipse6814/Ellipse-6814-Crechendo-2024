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
  public void initialize() {
    m_subsystem.resetEncoders();
  }


  @Override
  public void execute() {
    double leftSpeed = pidController.calculate(m_subsystem.getLeftEncoder() * ClimbConstants.kEncoderTicks2In, leftSetpoint);
    double rightSpeed = pidController.calculate(m_subsystem.getRightEncoder() * ClimbConstants.kEncoderTicks2In, rightSetpoint);

    m_subsystem.setLeftMotor(leftSpeed);
    m_subsystem.setRightMotor(rightSpeed);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.limitSwitches();
  }
}
