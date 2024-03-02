/*   Removed ELevator


package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.ConstantsMechanisms.ElevatorConstants.*;

import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ElevatorPIDCmd extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
//    private final PIDController pidController;
    private final SparkPIDController pidController;
    private double setpoint;
//    private double maxVel;
//    private double maxAcc;
    private double tolerance;

    private double elevMaxVel;
    private double elevMaxAcc;

    private boolean firstTime = true;

//     private final double elbowAngle;

    public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem,  double setpoint, double tolerance) {
        this.elevatorSubsystem = elevatorSubsystem;
//        this.elbowAngle = elbowAngle;

        this.pidController = elevatorSubsystem.getIntegratedSparkPID();
        this.pidController.setP(kP); 
        this.pidController.setI(kI);
        this.pidController.setD(kD);
        this.pidController.setFF(kFF);
        this.pidController.setOutputRange(kMinOutput, kMaxOutput);
        this.setpoint = setpoint;
//        this.maxVel = maxVel;
//        this.maxAcc = maxAcc;
        this.tolerance = tolerance;

        int smartMotionSlot = 0;
        this.pidController.setSmartMotionMaxVelocity(ElevatorConstants.kMaxVelDown, smartMotionSlot);
        this.pidController.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
        this.pidController.setSmartMotionMaxAccel(ElevatorConstants.kMaxAccDown, smartMotionSlot);
        this.pidController.setSmartMotionAllowedClosedLoopError(kAllowedErr, smartMotionSlot);


        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {

        this.pidController.setSmartMotionMaxVelocity(ElevatorConstants.kMaxVelDown, smartMotionSlot);
        this.pidController.setSmartMotionMaxAccel(ElevatorConstants.kMaxAccDown, smartMotionSlot);

//      System.out.println("ElevatorPIDCmd started!");
//      pidController.reset();
    }

    @Override
    public void execute() {

        if (firstTime) {

            double currentPosition = elevatorSubsystem.getEncoderInches();
            if(setpoint > currentPosition) 
                    {elevMaxVel = ElevatorConstants.kMaxVelUp; elevMaxAcc = ElevatorConstants.kMaxAccUp;}
            else    {elevMaxVel = ElevatorConstants.kMaxVelDown; elevMaxAcc = ElevatorConstants.kMaxAccDown;}
            this.pidController.setSmartMotionMaxVelocity(elevMaxVel, smartMotionSlot);
            this.pidController.setSmartMotionMaxAccel(elevMaxAcc, smartMotionSlot);

        // Using +/- 180 as inicator for small move up and down
        if(setpoint == 180.0) setpoint = currentPosition + ElevatorConstants.kSmallMoveInches;
        if(setpoint == -180.0) setpoint = currentPosition - ElevatorConstants.kSmallMoveInches;

        firstTime = false;
        }


        pidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Elevator Tolerance", this.tolerance);
    }

    @Override
    public void end(boolean interrupted) {
        firstTime = true;
 // TA 2/23/23 I think we need to comment this line out so elevator wont just "fall down"
        //       elevatorSubsystem.setMotor(0);
//        System.out.println("ElevatorPIDCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if (tolerance == 0.0) return false;               // hold elevator at cmded position until another command moves it
        else if (Math.abs((elevatorSubsystem.getEncoderInches() - setpoint)) < this.tolerance)   // else if within tolerance end Command 
        {
            firstTime = true;
            return true;
        }
    return false;

//        else return (Math.abs((elevatorSubsystem.getEncoderInches() - setpoint)) < this.tolerance);       // else if within tolerance end Command 
    }
}



*/