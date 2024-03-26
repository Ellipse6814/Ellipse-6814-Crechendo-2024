// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadband = 0.25;
  }
  
  public static class ShooterConstants {
    //motors assume that a negative value is clockwise direction,
    //so values should generally be negative
    public static final double kShooterIntakeSpeed = 0.55;
    public static final double kShooterSourceSpeed = 0.2;
    public static final double kShooterSpeakerAmpSpeed = 0.9;

    public static final int kBeamBreaker1Port = 4;
    public static final int kBeamBreaker2Port = 5;
    
    public static final int kMotor1Port = 14;
    public static final int kMotor2Port = 11;
    public static final int kMotor3Port = 17;
    public static final int kIntakeMotorPort = 18;

    //clockwise is positive:
    public static final boolean kMotor1Inverted = true;
    public static final boolean kMotor2Inverted = true;
    public static final boolean kMotor3Inverted = true;
    public static final boolean kIntakeMotorInverted = false;
      
    //Speaker auto lock constants  -refer to this diagram I drew https://prnt.sc/dl8j1ajQFxI2
    public static final double spkrHeight = 1.5; //do not set to 0   -causes division by 0 error
    public static final double spkrX = -16.2;
    public static final double spkrY = 0.6;
  }

  public static class ClimbConstants
  {
    public static final int kRightMotorPort = 28;
    public static final int kLeftMotorPort = 27;

    public static final int kLimitSwitchPort1 = 0;
    public static final int kLimitSwitchPort2 = 1;
    public static final int kLimitSwitchPort3 = 2;
    public static final int kLimitSwitchPort4 = 3;

    public static final double kp = 0.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    public static final double kEncoderTicks2Rot = 1/42;
    public static final double kMotorRot2ShaftRot = 1/3;
    public static final double kShaftRot2In = 1 / 12;
    public static final double kEncoderTicks2In = kEncoderTicks2Rot * kMotorRot2ShaftRot * kShaftRot2In;
  }

  public static class ArmConstants
  {
    public static final int kArmMotor1Port = 9;
    public static final int kArmMotor2Port = 10;

    public static final double kp = 0.4;
    public static final double ki = 0.0;
    public static final double kd = 0.05;

    //feedforward stuff
    public static final double ks = 0.04;
    public static final double kg = 0.36;
    public static final double kv = 1.20;
    public static final double kMaxVelocity = 0.025; //eli said this was max velocity idk

    public static final double kEncoderRotationsToRotations = 254/254;
    public static final double kEncoderTicks2Radians = kEncoderRotationsToRotations * Math.toRadians(360) * (1/94.5);

    //Change the influence of the PID controller and Feedforward controller
    //ex. pidInfluence = 0.0; feedforwardInfluence = 1.0;   means 100% feedforward (pid is not used)
    //    pidInfluence = 0.5; feedforwardInfluence = 0.5;   means half and half
    public static final double kPIDInfluence = 0.9;
    public static final double kFeedforwardInfluence = 0.1;
  }
  
  public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);  
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150.0/7);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

  public static final class DriveConstants {

        public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(19.75);
        // Distance between right and left wheels
        public static final double kWheelBase = edu.wpi.first.math.util.Units.inchesToMeters(21.75);
        // Distance between front and back wheels
        public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kDriveKinematics = new edu.wpi.first.math.kinematics.SwerveDriveKinematics(
                new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new edu.wpi.first.math.geometry.Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new edu.wpi.first.math.geometry.Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 7; //mod0
        public static final int kBackLeftDriveMotorPort = 1;//mod1
        public static final int kFrontRightDriveMotorPort = 5; //mod2
        public static final int kBackRightDriveMotorPort = 3;//mod3

        public static final int kFrontLeftTurningMotorPort = 8;//mod0
        public static final int kBackLeftTurningMotorPort = 2;//mod1
        public static final int kFrontRightTurningMotorPort = 6; //mod2
        public static final int kBackRightTurningMotorPort = 4;//mod3

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 3;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 0;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.182; //module 2
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.356; //module 3
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.618; //module 1
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 2.362; //module 0

        public static final double kPhysicalMaxSpeedMetersPerSecond = 8;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.25;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 2;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 1;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
