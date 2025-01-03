package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.ConstantsMechanisms.ElbowConstants.*;

import frc.robot.ConstantsMechanisms.ElbowConstants;

import frc.robot.subsystems.ElbowSubsystem;

import com.revrobotics.CANSparkMax;

import com.revrobotics.SparkPIDController;

public class ElbowPIDCmd extends Command {
    private final ElbowSubsystem elbowSubsystem;
    private final SparkPIDController pidController;

    private  double setpoint;
//    private double maxVel;
//    private double maxAcc;
    private double tolerance;
    private double elbowMaxVel;
    private double elbowMaxAcc;
    private boolean firstTime = true;

    public ElbowPIDCmd(ElbowSubsystem elbowSubsystem, double setpoint, double tolerance) {
        this.elbowSubsystem = elbowSubsystem;
        this.pidController = elbowSubsystem.getIntegratedSparkPID();

        this.pidController.setP(kP); 
        this.pidController.setI(kI);
        this.pidController.setD(kD);
        this.pidController.setFF(kFF);
        this.pidController.setOutputRange(kMinOutput, kMaxOutput);
        this.setpoint = setpoint;
        this.tolerance = tolerance;
//        this.maxVel = maxVel;
//        this.maxAcc = maxAcc;
        
        int smartMotionSlot = 0;
       this.pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
       this.pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);
       this.pidController.setSmartMotionMaxVelocity(ElbowConstants.kMaxVelDown, smartMotionSlot);
       this.pidController.setSmartMotionMaxAccel(ElbowConstants.kMaxAccDown, smartMotionSlot);

        addRequirements(elbowSubsystem);

    }
    



    @Override
    public void initialize() {
        firstTime = true;
        this.pidController.setSmartMotionMaxVelocity(ElbowConstants.kMaxVelDown, smartMotionSlot);
        this.pidController.setSmartMotionMaxAccel(ElbowConstants.kMaxAccDown, smartMotionSlot);

        //       System.out.println("elbowPIDCmd started!");
 //       pidController.reset();

    }

    @Override
    public void execute() {

        double currentAngle = elbowSubsystem.getEncoderDegrees();
        if (firstTime) {

            if(setpoint > currentAngle) 
                {elbowMaxVel = ElbowConstants.kMaxVelUp; elbowMaxAcc = ElbowConstants.kMaxAccUp;}
            else  {elbowMaxVel = ElbowConstants.kMaxVelDown; elbowMaxAcc = ElbowConstants.kMaxAccDown;}

            this.pidController.setSmartMotionMaxVelocity(elbowMaxVel, smartMotionSlot);
            this.pidController.setSmartMotionMaxAccel(elbowMaxAcc, smartMotionSlot);

            firstTime = false;
        }

        // Using +/- 180 as inicator for small move up and down
        if(setpoint == 180.0) setpoint = currentAngle + ElbowConstants.kSmallMoveDegrees;
        if(setpoint == -180.0) setpoint = currentAngle - ElbowConstants.kSmallMoveDegrees;

        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Elbow Tolerance", this.tolerance);

    }

    @Override
    public void end(boolean interrupted) {
        firstTime = true;
        elbowSubsystem.setMotor(0);
//        System.out.println("elbowPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if (this.tolerance == 0) return false;       // hold elevator at cmded position until another command moves it
        else if (Math.abs((elbowSubsystem.getEncoderDegrees() - setpoint)) < this.tolerance)   // else if within tolerance end Command 
            {
                firstTime = true;
                return true;
            }
        return false;
    }
}

