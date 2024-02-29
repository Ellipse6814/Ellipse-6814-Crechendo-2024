// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  public CANSparkMax motor1 = new CANSparkMax(ArmConstants.kArmMotor1Port, MotorType.kBrushless);
  public CANSparkMax motor2 = new CANSparkMax(ArmConstants.kArmMotor2Port, MotorType.kBrushless);
  public RelativeEncoder motor1Encoder = motor1.getEncoder();
  public RelativeEncoder motor2Encoder = motor2.getEncoder();

  public static ArmSubsystem instance;

  public static ArmSubsystem getInstance() {
    if (instance == null)
        instance = new ArmSubsystem();
    return instance;
  }

  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem() {

  }

  public void setMotor1(double speed){
    motor1.set(speed);
  }

  public void setMotor2(double speed){
    motor2.set(speed);
  }

  public void setMotors(double speed)
  {
    motor1.set(speed);
    motor2.set(speed);
  }
  
  public double getEncoderAverage()
  {
    return (motor1Encoder.getPosition() + motor2Encoder.getPosition()) / 2;
  }
  
  public void resetEncoders()
  {
    motor1Encoder.setPosition(0);
    motor2Encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
