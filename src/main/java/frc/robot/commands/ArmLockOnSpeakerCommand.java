// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmLockOnSpeakerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;

  private double setpoint;
  //WARNING: WE PROBABLY NEED A SUBSYSTEM OR SOMETHING IDK TO GET THESE VALUES WITH LIMELIGHT IDK HELP ME GRRRR
  private double spkDistace;
  private double spkHeight;

  private boolean limitArmTo90Degrees = false;

  private PIDController pidController = new PIDController(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd);
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv);
  
  public ArmLockOnSpeakerCommand(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;

    addRequirements(armSubsystem);
  }

 
  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    double currentArmRotation = armSubsystem.getEncoderAverage() * ArmConstants.kEncoderTicks2Radians;

    //Crazy math from WolframAlpha
    setpoint = 2 * (Math.atan((Math.sqrt(spkDistace*spkDistace  +  spkHeight * spkHeight) - spkDistace) / spkHeight) + Math.PI);

    if(setpoint > 90 && limitArmTo90Degrees) { setpoint = 90; } //Limit just in case (disable this for testing)
    SmartDashboard.putNumber("lock setpoint rad", setpoint);
    SmartDashboard.putNumber("lock setpoint deg", setpoint * (180 / Math.PI));
    
    //calculate outputs from pid and feedforward
    double pidOutput = pidController.calculate(currentArmRotation, setpoint);
    double feedforwardOutput = feedforward.calculate(setpoint, ArmConstants.kMaxVelocity);

    //add pid and feedforward outputs
    double setspeed = (ArmConstants.kPIDInfluence * pidOutput * 2) + (ArmConstants.kFeedforwardInfluence * feedforwardOutput);

    //kablooey
    //armSubsystem.setMotors(setspeed);
  }

  
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
