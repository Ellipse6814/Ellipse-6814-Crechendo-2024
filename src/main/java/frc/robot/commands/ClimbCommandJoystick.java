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

  private final Supplier<Double> leftMotorSupplier, rightMotorSupplier;
  
  public ClimbCommandJoystick(ClimbSubsystem subsystem, Supplier<Double> leftMotorSet, Supplier<Double> rightMotorSet) {
    m_subsystem = subsystem;
    this.leftMotorSupplier = leftMotorSet;
    this.rightMotorSupplier = rightMotorSet;
    addRequirements(subsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    double leftMotorSet = leftMotorSupplier.get();
    double rightMotorSet = rightMotorSupplier.get();

    m_subsystem.setLeftMotor(leftMotorSet);
    m_subsystem.setRightMotor(rightMotorSet);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
