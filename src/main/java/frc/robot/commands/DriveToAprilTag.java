// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.BlinkinLEDController;
import frc.robot.subsystems.BlinkinLEDController.BlinkinPattern;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToAprilTag extends Command {
  private final Swerve s_Swerve;
  boolean done = false; 
  int  pipeline;
  double aprilTagOffset;
  private BlinkinLEDController m_ledCont;

  /** Creates a new CenterAprilTag. */
  public DriveToAprilTag(Swerve swerve, int pipeline, double aprilTagOffset, BlinkinLEDController ledCont) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Swerve = swerve;
    addRequirements(s_Swerve);
    this.aprilTagOffset = aprilTagOffset;
    this.pipeline = pipeline;
    m_ledCont = ledCont;
    addRequirements(m_ledCont);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

//    LimelightHelpers.setPipelineIndex("limelight",pipeline);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");
  //   NetworkTableEntry tx = table.getEntry("tx");
    double valid = tv.getDouble(0.0);
  //   double horizontalOffset = tx.getDouble(0.0);

    double xPower, yPower, rotatePower;
  //   double horizontalOffsetThreshold = 5.0;

    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    double xOffsetInches = botpose[0] * 39.3701;
 //   double yOffsetInches = botpose[1] * 39.3701   // we want oour offset 36 inches away from robot in Y direction
    double zOffsetInches = botpose[2] * 39.3701 + aprilTagOffset;
   //  double pitchDegrees = botpose[3]) ;
    double yawDegrees =botpose[4] ;
 //  double rollDegrees = botpose[5];

 
    if (valid == 1.0) { // Execute only if Limelight sees valid target

      double kXsign = xOffsetInches/Math.abs(xOffsetInches);
      if (Math.abs(xOffsetInches) > 24) xPower = -0.2*kXsign; 
      else if (Math.abs(xOffsetInches) > 6) xPower = -0.1*kXsign;
      else if (Math.abs(xOffsetInches) > 2) xPower = -0.04*kXsign;
      else xPower = 0.0;


      double kZsign = zOffsetInches/Math.abs(zOffsetInches);
      if (Math.abs(zOffsetInches) > 24) yPower = -0.4*kZsign; 
      else if (Math.abs(zOffsetInches) > 6) yPower = -0.2*kZsign;
      else if (Math.abs(zOffsetInches) > 1) yPower = -0.05*kZsign;
      else yPower = 0.0;




      double kYawsign = yawDegrees/Math.abs(yawDegrees);
      if (Math.abs(yawDegrees) > 24) rotatePower = -0.2*kYawsign; 
      else if (Math.abs(yawDegrees) > 6) rotatePower = -0.1*kYawsign;
      else if (Math.abs(yawDegrees) > 1) rotatePower = -0.04*kYawsign;
      else rotatePower = 0.0;
//      double kYawDeg = 100.0;
//      if (Math.abs(yawDegrees) >kYawDeg) rotatePower = -yawDegrees/Math.abs(yawDegrees); else rotatePower = -yawDegrees/kYawDeg;
//      rotatePower = MathUtil.applyDeadband(rotatePower, .02);

      done = false;
      if ((xPower==0)  &&  (yPower==0)  && (rotatePower == 0)) done = true; 


      if (done) m_ledCont.setPattern(BlinkinPattern.WHITE);
      else if (yPower!=0) m_ledCont.setPattern(BlinkinPattern.RED);
      else if (rotatePower!=0) {if (rotatePower> 0) m_ledCont.LED_TurnLeft(); else m_ledCont.LED_TurnRight(); }
      else m_ledCont.setPattern(BlinkinPattern.YELLOW);



      s_Swerve.drive(
        new Translation2d(-yPower, xPower).times(Constants.Swerve.maxSpeed), 
        -rotatePower * Constants.Swerve.maxAngularVelocity, 
        false,                   // TA TODO: Probably want false here!!!! (was true)
        true);
      }
    else {
      s_Swerve.stop();
      done = true;
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
