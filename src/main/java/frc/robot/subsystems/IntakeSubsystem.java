// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
  public static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null)
        instance = new IntakeSubsystem();
    return instance;
  }
  public CANSparkMax motor;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    motor = new CANSparkMax(ShooterConstants.kIntakeMotorPort, MotorType.kBrushless);
    motor.setInverted(ShooterConstants.kIntakeMotorInverted);
  }


  public void setspeed(double speed){
    motor.set(speed);
  }

  public void stop()
  {
    motor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
