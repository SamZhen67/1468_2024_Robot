// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SlideToAprilTag extends Command {
  private final Swerve s_Swerve;

  /** Creates a new CenterAprilTag. */
  public SlideToAprilTag(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    addRequirements(s_Swerve);
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

    double speedPercent = 0.1;
    double horizontalOffsetThreshold = 5.0;

    if (valid == 1) { // Execute only if Limelight sees valid target
      if (Math.abs(horizontalOffset) > horizontalOffsetThreshold) {
        if (horizontalOffset < 0) { // Slide left
          s_Swerve.drive(
            new Translation2d(0, speedPercent).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
          );
        }
        else if (horizontalOffset >= 0) { // Slide right
          s_Swerve.drive(
            new Translation2d(0, -speedPercent).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
          );
        }
      }
      else {
        s_Swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
          );
      }
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
