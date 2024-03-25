package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
    public static LimelightSubsystem instance;

    public static LimelightSubsystem getInstance() {
      if (instance == null)
          instance = new LimelightSubsystem();
      return instance;
    }

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry botpose = table.getEntry("botpose");

    public LimelightSubsystem () {
    }

    public double getBotPoseTableEntry(int index) {
        // Index 0 is x, Index 1 is y, Index 5 is yaw
        return botpose.getDoubleArray(new double[8])[index];
    }

}