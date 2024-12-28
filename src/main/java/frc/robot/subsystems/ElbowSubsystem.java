// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.ElbowConstants.*;

//import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class ElbowSubsystem extends SubsystemBase {

    private CANSparkMax leftElbowMotor = new CANSparkMax(kLeftElbowMotorPort, CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder elbowEncoder =  leftElbowMotor.getEncoder();//
    private CANSparkMax rightElbowMotor = new CANSparkMax(kRightElbowMotorPort, CANSparkMax.MotorType.kBrushless);
 
    public ElbowSubsystem() {
        rightElbowMotor.follow(leftElbowMotor, true);       // TA TODO: polarity of these?
        leftElbowMotor.setInverted(false);
        rightElbowMotor.setInverted(true);              // TA TODO: Are both inverts needed
      
        elbowEncoder.setPosition(kZeroOffset);
        elbowEncoder.setPositionConversionFactor(kEncoderRotation2Degrees);
        leftElbowMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kForwardSoftLimit);
        leftElbowMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kReverseSoftLimit);
        leftElbowMotor.setSmartCurrentLimit(60);
        rightElbowMotor.setSmartCurrentLimit(60);
        // DOc says use coast mode for RPM control - brake will fight control - TODO: TA - try this out, change back if no good
//        leftElbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
//        rightElbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftElbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightElbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
// Ks is a starting point of 
//    final ArmFeedforward m_feedforward = new ArmFeedforward(1.0, 1.12, 1.95, 0.12);


    }

    @Override
    public void periodic() {
       log();
    }


    public void setCoastMode() {
      leftElbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightElbowMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setBrakeMode() {
      leftElbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightElbowMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

 
    public void resetElbowEncGyro() {
      elbowEncoder.setPosition(kZeroOffset);

    }

    
    public SparkPIDController getIntegratedSparkPID( ) {
        return leftElbowMotor.getPIDController();
     }
 

    public void setMotor(double speed) {
        leftElbowMotor.set(speed);
     }


    /** Set the elevator motor to move in the up direction. */
    public void elbowUp() {
        leftElbowMotor.set(kUpSpeed);
     }
 
      /** Set the elevator motor to move in the down direction. */
      public void elbowDown() {
        leftElbowMotor.set(kDownSpeed);
      }
 
     /** Set the elevator motor to move in the up direction. */
     public void elbowStop() {
        leftElbowMotor.set(0);
     }
 

    public double getEncoderDegrees() {
        return elbowEncoder.getPosition();
    }

// Moving the arm down increases the angle value
    public double getSmallDownAngle() {
       double temp = getEncoderDegrees();
      if ( (kMaxAngle -  temp) > kSmallMoveDegrees)
         return (temp + kSmallMoveDegrees );
      else return (kMaxAngle - 1.0) ;    // dont go past Max
   }
// Moving the arm up decreases the angle value
   public double getSmallUpAngle() {
    double temp = getEncoderDegrees();
    if ( (temp - kStartAngle) > kSmallMoveDegrees)
        return (temp - kSmallMoveDegrees );
    else return (kStartAngle) ;          // dont go past min / start angle
   }
   
   
    /** Return true when the claw Opened limit switch triggers. */
    public boolean isAtMax() {
//        return m_ElbowAtMaxAngle.get();
        return  ( (elbowEncoder.getVelocity() > 0.01) && (this.getEncoderDegrees() > kForwardSoftLimit)   );

    }
    /** Return true when the claw Closed limit switch triggers. */
    public boolean isAtMin() {
  //      return m_ElbowAtMinAngle.get();
        return  ( (elbowEncoder.getVelocity() < -0.01) && (this.getEncoderDegrees() < kReverseSoftLimit)   );
    }

    public void log() {
        SmartDashboard.putNumber("Elbow Encoder degrees", getEncoderDegrees());
        SmartDashboard.putNumber("Elbow Lt Temperature", getLtTemp());
        SmartDashboard.putNumber("Elbow Rt Temperature", getRtTemp());
    }
    
  /** Get Temp. */
  public double getLtTemp() {
    return leftElbowMotor.getMotorTemperature();
  }
  public double getRtTemp() {
    return rightElbowMotor.getMotorTemperature();
  }

}




