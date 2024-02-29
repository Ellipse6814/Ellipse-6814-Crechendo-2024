// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmRaiseCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  private double setpoint;

  private PIDController pidController = new PIDController(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd);
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv);
  
  public ArmRaiseCommand(ArmSubsystem armSubsystem, double setpoint) {
    m_subsystem = armSubsystem;
    this.setpoint = setpoint; 

    addRequirements(armSubsystem);
  }

 
  @Override
  public void initialize() {}


  @Override
  public void execute() {
    double currentdistance = m_subsystem.getEncoderAverage() * ArmConstants.kEncoderTicks2Radians;
    
    //calculate outputs from pid and feedforward
    double pidOutput = pidController.calculate(currentdistance);
    double feedforwardOutput = feedforward.calculate(setpoint, ArmConstants.kMaxVelocity);

    //add pid and feedforward outputs
    double setspeed = (ArmConstants.kPIDInfluence * pidOutput) + (ArmConstants.kFeedforwardInfluence * feedforwardOutput);

    //kablooey
    m_subsystem.setMotors(setspeed);
  }

  
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
