// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToAprilTagPP extends Command {
  private final Swerve s_Swerve;
  boolean done = false;
  int pipeline;
  double aprilTagOffset;
  double robotYawDegs, turnRads;

  /** Creates a new CenterAprilTag. */
  public DriveToAprilTagPP(Swerve swerve, int pipeline, double aprilTagOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    addRequirements(s_Swerve);
    this.aprilTagOffset = aprilTagOffset;
    this.pipeline = pipeline;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotYawDegs = s_Swerve.getYawDeg();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

    // LimelightHelpers.setPipelineIndex("limelight",pipeline);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
    // NetworkTableEntry tx = table.getEntry("tx");
    double valid = tv.getDouble(0.0);
    // double horizontalOffset = tx.getDouble(0.0);

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace")
        .getDoubleArray(new double[6]);

    double x = botpose[0];
    // double yOffsetInches = botpose[1] * 39.3701 // we want oour offset 36 inches
    // away from robot in Y direction
    double y = botpose[2] + aprilTagOffset / 39.3701;
    // double pitchDegrees = botpose[3]) ;
    double yawDegrees = botpose[4];
    // double rollDegrees = botpose[5];
    Rotation2d turn2D =  Rotation2d.fromDegrees(-yawDegrees);

    if (valid == 1.0) { // Execute only if Limelight sees valid target

      Pose2d currentPose = s_Swerve.getPose();


      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
//      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-x, y)), new Rotation2d()); // was
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(-y, -x)), turn2D); // was
                                                                                                                 // x,-y

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              4.0, 4.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, (turn2D)) // was - now +
      );

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();

      //////// Have to get initial yaw value from robot and calculate a delta yaw offf
      //////// the apriltag yaw

    } else {
      s_Swerve.stop();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
