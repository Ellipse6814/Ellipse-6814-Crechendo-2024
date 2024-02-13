// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {



  }


  public void setRightMotor1(double speed){

  }

  public void setLeftMotor(double speed){


  }
  
  public double getLeftEncoder(){
    return 0.0;
  }

  public double getRightEncoder(){
    return 0.0;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
