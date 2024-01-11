package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    public void printLeft() {
        SmartDashboard.putString("Slide Direction", "Left");
    }

    public void printRight() {
        SmartDashboard.putString("Slide Direction", "Right");
    }

    public void printStop() {
        SmartDashboard.putString("Slide Direction", "STOP");
    }

    public void printNoValidTargets() {
        SmartDashboard.putString("Slide Direction", "PRESSED, No valid targets");
    }

    public void centerAprilTagRelease() {
        SmartDashboard.putString("Slide Direction", "NOT ACTIVE");
    }

    @Override
    public void periodic(){
        /// Limelight SmartDashboard
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");

        //read values periodically
        double valid = tv.getDouble(0.0);
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("ValidTarget", valid);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
}
