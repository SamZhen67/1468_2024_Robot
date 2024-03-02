package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ConstantsMechanisms.LimelightConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerveRotatetoSpeakerAprilTag extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private double rotationVal;

    public TeleopSwerveRotatetoSpeakerAprilTag(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {

    //   LimelightHelpers.setPipelineIndex("limelight",LimelightConstants.SPEAKER_PIPELINE);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(LimelightConstants.SPEAKER_PIPELINE);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = table.getEntry("tv");
        double valid = tv.getDouble(0.0);
        NetworkTableEntry tx = table.getEntry("tx");
        double angle = tx.getDouble(0.0);

//        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace").getDoubleArray(new double[6]);

//        double xOffsetInches = botpose[0] * 39.3701;
//        double zOffsetInches = botpose[2] * 39.3701;
//        double angle = Math.atan2(-zOffsetInches,-xOffsetInches) * 180 / Math.PI;
 //       double angle = Math.atan2(-xOffsetInches,-zOffsetInches) * 180 / Math.PI;

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
         
//        if(valid == 1.0) rotationVal = angle /120.0;     // yaw is +/-180  - 60 way too fast but not working at all
        if(valid == 1.0) rotationVal = -angle /100.0;     // 
        else  rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}