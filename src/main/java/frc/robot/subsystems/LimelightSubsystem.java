package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    public static LimelightSubsystem instance;
    private static final double[] defaultArray = new double[] { 6814.6814, 6814.6814, 0, 0, 0, 6814.6814, 0, 0 };

    public static LimelightSubsystem getInstance() {
      if (instance == null)
          instance = new LimelightSubsystem();
      return instance;
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry botpose = table.getEntry("botpose");
    NetworkTableEntry redBotpose = table.getEntry("botpose_wpired");
    NetworkTableEntry blueBotpose = table.getEntry("botpose_wpiblue");

    public LimelightSubsystem () {
    }

    public double getBotPoseTableEntry(int index) {
        // Index 0 is x, Index 1 is y, Index 5 is yaw
        return botpose.getDoubleArray(defaultArray)[index];
    }
    public double getRedBotPoseTableEntry(int index) {
        // Index 0 is x, Index 1 is y, Index 5 is yaw
        return redBotpose.getDoubleArray(defaultArray)[index];
    }
    public double getBlueBotPoseTableEntry(int index) {
        // Index 0 is x, Index 1 is y, Index 5 is yaw
        return blueBotpose.getDoubleArray(defaultArray)[index];
    }

    public double getMyTeamBotposeTableEntry(int index)
    {
        if(DriverStation.getAlliance().get() == Alliance.Red) 
        { return getRedBotPoseTableEntry(index); }
        else 
        { return getBlueBotPoseTableEntry(index); }
    }

    public double getOppositeTeamBotposeTableEntry(int index)
    {
        if(DriverStation.getAlliance().get() == Alliance.Blue) 
        { return getRedBotPoseTableEntry(index); }
        else 
        { return getBlueBotPoseTableEntry(index); }
    }
}