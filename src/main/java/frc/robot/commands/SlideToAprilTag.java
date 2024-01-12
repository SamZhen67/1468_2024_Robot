// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  //   NetworkTableEntry tx = table.getEntry("tx");
     double valid = tv.getDouble(0.0);
  //   double horizontalOffset = tx.getDouble(0.0);

     double xPower, yPower;
  //   double horizontalOffsetThreshold = 5.0;

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

   double xOffsetInches = botpose[0] * 39.3701;
   double yOffsetInches =botpose[2] * 39.3701 + 36.0;   // we want oour offset 36 inches away from robot in Y direction
  double zOffsetInches =botpose[1] * 39.3701;
   double rollDegrees = botpose[3]* (180/Math.PI) ;
   double pitchDegrees = botpose[4]* (180/Math.PI) ;
    double yawDegrees =botpose[5] * (180/Math.PI);

    if (Math.abs(xOffsetInches) >50) xPower = -.5*xOffsetInches/xOffsetInches; else xPower = xOffsetInches/100.0;
    xPower = MathUtil.applyDeadband(xPower, .025);
     if (Math.abs(yOffsetInches) >50) yPower = -.5*yOffsetInches/yOffsetInches; else yPower = -yOffsetInches/100.0;
    yPower = MathUtil.applyDeadband(yPower, .025);


    if (valid == 1.0) { // Execute only if Limelight sees valid target
    
          s_Swerve.drive(
            new Translation2d(xPower, yPower).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true);

        
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


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
