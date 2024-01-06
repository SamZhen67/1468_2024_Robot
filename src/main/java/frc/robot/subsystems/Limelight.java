package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
    private Swerve s_Swerve;

    // Slide left or right until AprilTag is centered in Limelight 
    public void centerAprilTag() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry tx = table.getEntry("tx");
        double valid = tv.getDouble(0.0);
        double horizontalOffset = tx.getDouble(0.0);

        // Adjust as needed
        double speedPercent = 0.1;
        double horizontalOffsetThreshold = 1.0;

        if (valid == 1) { // Execute only if Limelight sees valid target
            if (Math.abs(horizontalOffset) > horizontalOffsetThreshold) {
                if (horizontalOffset < 0) { // Slide left

                }
                else if (horizontalOffset >= 0) { // Slide right

                }
            }
        } 
    }
}
