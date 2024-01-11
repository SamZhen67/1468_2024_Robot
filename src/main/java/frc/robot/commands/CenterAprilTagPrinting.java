// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class CenterAprilTagPrinting extends Command {
  private final Limelight s_Limelight;

  /** Creates a new CenterAprilTag. */
  public CenterAprilTagPrinting(Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Limelight = limelight;
    addRequirements(s_Limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    double valid = tv.getDouble(0.0);
    double horizontalOffset = tx.getDouble(0.0);

    double horizontalOffsetThreshold = 5.0;

    if (valid == 1) { // Execute only if Limelight sees valid target
      if (Math.abs(horizontalOffset) > horizontalOffsetThreshold) {
        if (horizontalOffset < 0) { // Slide left
          s_Limelight.printLeft();
        }
        else if (horizontalOffset >= 0) { // Slide right
          s_Limelight.printRight();
        }
      }
      else {
        s_Limelight.printStop();
      }
    } 
    else {
      s_Limelight.printNoValidTargets();
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Limelight.centerAprilTagRelease();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
