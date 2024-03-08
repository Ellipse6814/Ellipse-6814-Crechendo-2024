// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ClimbCommandJoystick extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimbSubsystem m_subsystem;

  private final double leftMotorSet, rightMotorSet;
  
  public ClimbCommandJoystick(ClimbSubsystem subsystem, Supplier<Double> leftMotorSet, Supplier<Double> rightMotorSet) {
    m_subsystem = subsystem;
    this.leftMotorSet = leftMotorSet.get();
    this.rightMotorSet = rightMotorSet.get();
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    m_subsystem.setLeftMotor(leftMotorSet * 0.3);
    m_subsystem.setRightMotor(rightMotorSet * 0.3);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
