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
import edu.wpi.first.wpilibj.DigitalInput;


public class ClimbSubsystem extends SubsystemBase {
  public CANSparkMax leftMotor = new CANSparkMax(ClimbConstants.kLeftMotorPort, MotorType.kBrushless);
  public CANSparkMax rightMotor = new CANSparkMax(ClimbConstants.kRightMotorPort, MotorType.kBrushless);
  public RelativeEncoder leftEncoder = leftMotor.getEncoder();
  public RelativeEncoder rightEncoder = rightMotor.getEncoder();

  private final DigitalInput limitSwitch1 = new DigitalInput(ClimbConstants.kLimitSwitchPort1);
  private final DigitalInput limitSwitch2 = new DigitalInput(ClimbConstants.kLimitSwitchPort2);
  private final DigitalInput limitSwitch3 = new DigitalInput(ClimbConstants.kLimitSwitchPort3);
  private final DigitalInput limitSwitch4 = new DigitalInput(ClimbConstants.kLimitSwitchPort4);



  public static ClimbSubsystem instance;

  public static ClimbSubsystem getInstance() {
    if (instance == null)
        instance = new ClimbSubsystem();
    return instance;
  }

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

  public void resetEncoders(){
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  public boolean limitSwitches(){
    return limitSwitch1.get() || limitSwitch2.get() || limitSwitch3.get() || limitSwitch4.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
