// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  public CANSparkMax leftMotor = new CANSparkMax(ClimbConstants.kLeftMotorPort, MotorType.kBrushless);
  public CANSparkMax rightMotor = new CANSparkMax(ClimbConstants.kRightMotorPort, MotorType.kBrushless);
  public RelativeEncoder leftEncoder = leftMotor.getEncoder();
  public RelativeEncoder rightEncoder = rightMotor.getEncoder();

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {

  }

  public void setRightMotor(double speed){
    rightMotor.set(speed);
  }

  public void setLeftMotor(double speed){
    leftMotor.set(speed);
  }
  
  public double getLeftEncoder(){
    return leftEncoder.getPosition();
  }

  public double getRightEncoder(){
    return rightEncoder.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
