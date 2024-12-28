// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveToNote extends Command {
  private final Swerve s_Swerve;
  boolean done = false; 
  int  pipeline;
  /** Creates a new DriveToNote. */
  public DriveToNote(Swerve swerve, int pipeline) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    addRequirements(s_Swerve);
    this.pipeline = pipeline;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    NetworkTableInstance.getDefault().getTable("limelight-note").getEntry("pipeline").setNumber(pipeline);

//    LimelightHelpers.setPipelineIndex("limelight",pipeline);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-note");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    double valid = tv.getDouble(0.0);
  //   double horizontalOffset = tx.getDouble(0.0);

    double xPower, yPower, rotatePower;
  //   double horizontalOffsetThreshold = 5.0;

    //double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-note").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    double xOffsetInches = tx.getDouble(0.0) * 39.3701;
    
 //   double yOffsetInches = botpose[1] * 39.3701   // we want oour offset 36 inches away from robot in Y direction
    double yOffsetInches = ty.getDouble(0.0) * 39.3701;
   //  double pitchDegrees = botpose[3]) ;
    //double yawDegrees =botpose[4] ;
 //  double rollDegrees = botpose[5];

    // 75 overshoots  (higher is slower)
    double kXinch = 100.0;
    if (Math.abs(xOffsetInches) >kXinch) xPower = -(kXinch/100.0)*xOffsetInches/Math.abs(xOffsetInches); else xPower = -xOffsetInches/kXinch;
    xPower = MathUtil.applyDeadband(xPower, .005);

    // 75 overshoots  (higher is slower)
    double kYinch = 100.0;
    if (Math.abs(yOffsetInches) >kYinch) yPower = -(kYinch/100.0)*yOffsetInches/Math.abs(yOffsetInches); else yPower = -yOffsetInches/kYinch;
    yPower = MathUtil.applyDeadband(yPower, .005);

    done = false;
    if ((xPower==0)  &&  (yPower==0)) done = true; 


    if (valid == 1.0) { // Execute only if Limelight sees valid target
      s_Swerve.drive(
//        new Translation2d(xPower, yPower).times(Constants.Swerve.maxSpeed), 
        new Translation2d(-xPower, yPower).times(Constants.Swerve.maxSpeed), 
        0, 
        false,                   // TA TODO: Probably want false here!!!! (was true)
        true);
      }
    else {
      s_Swerve.stop();
//      s_Swerve.drive(
//        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
//        0 * Constants.Swerve.maxAngularVelocity, 
//        false, 
//        true
//      );
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
