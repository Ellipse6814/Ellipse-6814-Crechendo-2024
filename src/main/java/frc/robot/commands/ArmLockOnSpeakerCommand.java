// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmLockOnSpeakerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private double setpoint;

  private final double speakerXPos; //limelight field coordinates
  private final double speakerYPos;
  private double spkDistace;
  private double spkHeight;

  private boolean limitArmTo70Degrees = true;

  private PIDController pidController = new PIDController(ArmConstants.kp, ArmConstants.ki, ArmConstants.kd);
  private ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv);
  
  public ArmLockOnSpeakerCommand(ArmSubsystem armSubsystem, LimelightSubsystem limelightSubsystem) {
    this.armSubsystem = armSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    speakerXPos = ShooterConstants.spkrX;
    speakerYPos = ShooterConstants.spkrY;
    spkHeight = ShooterConstants.spkrHeight;

    addRequirements(armSubsystem);
  }

 
  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    //----Calculate distance to speaker----
    double botX = limelightSubsystem.getMyTeamBotposeTableEntry(0); // get bot field position
    double botY = limelightSubsystem.getMyTeamBotposeTableEntry(1); // UNTESTED PLS TEST ME!!!!

    double xDifference = botX - speakerXPos;
    double yDifference = botY - speakerYPos;

    spkDistace = Math.sqrt(xDifference*xDifference  +  yDifference*yDifference); //thank you pythagoras

    double setpointDegrees = 6814;

    //----Calculate arm rotation using speaker distance we just calculated up there above me look up there! woah!!----
    double currentArmRotation = armSubsystem.getEncoderAverage() * ArmConstants.kEncoderTicks2Radians;

    if(botX == 6814.6814 || botY == 6814.6814) //if limelight cannot see apriltag, set setpoint = 0
    {
      setpoint = 0;
    }
    else
    {
      //Crazy math from WolframAlpha
      setpoint = (2 * (Math.atan((Math.sqrt(spkDistace*spkDistace  +  spkHeight * spkHeight) - spkDistace) / spkHeight) + Math.PI)) % Math.toRadians(360);
      //setpoint = Math.atan2(yDifference, xDifference);  BAD DONT USE THIS
      setpoint = Math.toRadians(40) - setpoint; //Because, when arm angle goes up, shooting angle goes down se we gotta reverse
                             //this i am making no sense im tired pls help me >_<
      setpointDegrees = Math.toDegrees(setpoint);

      if(setpointDegrees > 70 && limitArmTo70Degrees) { setpoint = Math.toRadians(70); } //Limit just in case (disable this for testing)
      if(setpointDegrees < 0) { setpoint = 0; }

      setpointDegrees = Math.toDegrees(setpoint); //for logging purposes
    }

    SmartDashboard.putNumber("lock setpoint deg", setpointDegrees);
    
    //calculate outputs from pid and feedforward
    double pidOutput = pidController.calculate(currentArmRotation, setpoint);
    double feedforwardOutput = feedforward.calculate(setpoint, ArmConstants.kMaxVelocity);

    //add pid and feedforward outputs
    double setspeed = (ArmConstants.kPIDInfluence * pidOutput * 2) + (ArmConstants.kFeedforwardInfluence * feedforwardOutput);

    //kablooey
    armSubsystem.setMotors(setspeed);
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
