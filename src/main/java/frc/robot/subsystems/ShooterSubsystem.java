// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final DigitalInput beamBreaker1;
  private final DigitalInput beamBreaker2;
  
  private final CANSparkMax motor1;
  private final CANSparkMax motor2;
  private final CANSparkMax motor3;
 

  public ShooterSubsystem() {
    beamBreaker1 = new DigitalInput(ShooterConstants.kBeamBreaker1Port);
    beamBreaker1 = new DigitalInput(ShooterConstants.kBeamBreaker2Port);

    motor1 = new CANSparkMax(ShooterConstants.kMotor1Port, MotorType.kBrushless);    
    motor2 = new CANSparkMax(ShooterConstants.kMotor2Port, MotorType.kBrushless);
    motor2 = new CANSparkMax(ShooterConstants.kMotor3Port, MotorType.kBrushless);
  }

  public void setMotor1(double speed){
    motor1.set(speed);
  }

  public void setMotor2(double speed){
    motor2.set(speed);
  }

  public void setMotor3(double speed){
    motor3.set(speed);
  }
  
  public boolean getSensor1(){
    return !beamBreaker1.get();
  }

  public boolean getSensor2(){
    return !beamBreaker2.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
