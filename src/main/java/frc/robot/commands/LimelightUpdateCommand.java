package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class LimelightUpdateCommand extends Command {

    private final LimelightSubsystem m_Limelight;
    private final SwerveSubsystem m_Swerve;

    public LimelightUpdateCommand (LimelightSubsystem limelightSubsystem, SwerveSubsystem swervesubsystem) {

        m_Limelight = limelightSubsystem;
        m_Swerve = swervesubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Rotation2d rotation = new Rotation2d(Math.toRadians(m_Limelight.getBotPoseTableEntry(0)));
        Pose2d pose = new Pose2d(m_Limelight.getBotPoseTableEntry(0), m_Limelight.getBotPoseTableEntry(1), rotation);

        if (m_Limelight.getBotPoseTableEntry(7) == 2) {
            m_Swerve.resetOdometry(pose);
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
} 