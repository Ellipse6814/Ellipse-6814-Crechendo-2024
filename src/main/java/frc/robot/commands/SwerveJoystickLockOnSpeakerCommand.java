package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickLockOnSpeakerCommand extends Command {
    private final ArmSubsystem m_ArmSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double turningSpeed = 0.0;

    public SwerveJoystickLockOnSpeakerCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction,
            Supplier<Boolean> fieldOrientedFunction, ArmSubsystem m_ArmSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.m_ArmSubsystem = m_ArmSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        //============== regular swerve code ==============
        // 1. Get real-time joystick inputs
        if(m_ArmSubsystem.getEncoderAverage() * Constants.ArmConstants.kEncoderTicks2Radians > Math.toRadians(30)){
            xSpeed = xSpdFunction.get() * OIConstants.slowmodeMultiplier;
            ySpeed = ySpdFunction.get() * OIConstants.slowmodeMultiplier;

        } else{
            xSpeed = xSpdFunction.get();
            ySpeed = ySpdFunction.get();
        }
        SmartDashboard.putBoolean("slowmode", m_ArmSubsystem.getEncoderAverage() > 30);
        SmartDashboard.putNumber("slowmode calc", m_ArmSubsystem.getEncoderAverage() * Constants.ArmConstants.kEncoderTicks2Radians);

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

        //============== MY (andrew's) MODIFICATION ==============

        double botX = limelightSubsystem.getBotPoseTableEntry(0); // get bot field position
        double botY = limelightSubsystem.getBotPoseTableEntry(1);
        double botYaw = limelightSubsystem.getOppositeTeamBotposeTableEntry(5);

        double xDifference = botX - ShooterConstants.spkrX;
        double yDifference = botY - ShooterConstants.spkrY;

        double angleDistance = Math.atan2(yDifference, xDifference) - Math.toRadians(botYaw);
        double sign = angleDistance < 0 ? -1 : 1;

        double smoothingMultiplier = Math.toDegrees(angleDistance) / 6;
        if(Math.abs(smoothingMultiplier) > 1)
        {
            smoothingMultiplier = 1;
        }

        if(botX == 6814.6814) //if bot cannot see AprilTag, do not turn the bot
        {
            turningSpeed = 0;
        }
        else
        {
            turningSpeed = -sign * 0.7501 * Math.abs(smoothingMultiplier);
        }

        /*  No longer needed
        SmartDashboard.putNumber("turningspeed", turningSpeed);
        SmartDashboard.putNumber("angle distance", Math.toDegrees(angleDistance));
        SmartDashboard.putNumber("bot yaw", botYaw);
        */

        //============== regular swerve code ==============
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}