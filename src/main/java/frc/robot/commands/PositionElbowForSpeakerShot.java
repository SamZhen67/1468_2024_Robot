// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//import frc.robot.Constants;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.subsystems.ElbowSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PositionElbowForSpeakerShot extends Command {
  private  ElbowSubsystem s_Elbow;

  /** Creates a new CenterAprilTag. */
  public PositionElbowForSpeakerShot(ElbowSubsystem elbow) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Elbow = elbow;
    addRequirements(s_Elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tz = table.getEntry("tx");
    double valid = tv.getDouble(0.0);
    double zDistanceInInches = tz.getDouble(0.0);


  //   double horizontalOffsetThreshold = 5.0;

 //   double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

 //   double zDistanceInInches = -botpose[2] * 39.3701;
 
    double elbowAngle = .46875 * zDistanceInInches - 9.0625;

    if (valid == 1.0) { // Execute only if Limelight sees valid target
//    if (false) { // Execute only if Limelight sees valid target
         new ElbowPIDCmd(s_Elbow, elbowAngle, 0.0);
      }
    else {
         new ElbowPIDCmd(s_Elbow, ElbowConstants.kScoreInSpeakerFromPodiumAngle, 0.0);
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
