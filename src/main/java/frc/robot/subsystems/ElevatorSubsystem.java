   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax leftElevatorMotor = new CANSparkMax(kLeftElMotorPort, CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder elevatorEncoder =  leftElevatorMotor.getEncoder();//
    private CANSparkMax rightElevatorMotor = new CANSparkMax(kRightElMotorPort, CANSparkMax.MotorType.kBrushless);
 
    public ElevatorSubsystem() {
        leftElevatorMotor.setInverted(false);            // TA TODO: polarity of these?
        rightElevatorMotor.setInverted(true);
        rightElevatorMotor.follow(leftElevatorMotor, true);    // TA TODO: check if both inverts are needed
        
        elevatorEncoder.setPosition(kZeroOffset);     // TA TODO: Determine offset
        elevatorEncoder.setPositionConversionFactor(kEncoderRotation2Inches); // TA TODO: Determine conversion factor
        leftElevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kForwardSoftLimit);
        leftElevatorMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kReverseSoftLimit);

      }

    @Override
    public void periodic() {
       log();
    }

    public SparkPIDController getIntegratedSparkPID( ) {
        return leftElevatorMotor.getPIDController();
     }
    
    
    public void setMotorSpeed(double speed) {
       leftElevatorMotor.set(speed);
    }


    public void elevatorUp( ) {
       leftElevatorMotor.set(kUpSpeed);
    }

 
     public void elevatorDown() {
        leftElevatorMotor.set(kDownSpeed);
     }

  
    public void elevatorStop() {
        leftElevatorMotor.set(0);
    }


    public double getEncoderInches() {
      return elevatorEncoder.getPosition() ;
  }

  public double getSmallUpLocation() {
   double temp = getEncoderInches();
   if ( (kForwardSoftLimit -  temp) > kSmallMoveInches)
      return (temp + kSmallMoveInches );
   else return (kForwardSoftLimit - 1.0) ;
}

public double getSmallDownLocation() {
   double temp = getEncoderInches();
   if ( (temp) > kSmallMoveInches)
      return (temp - kSmallMoveInches );
   else return (kHomePosition) ;
}

    /** Return true when the elevator is at the top. */
    public boolean isAtTop() {
 //       return m_ElevatorAtTop.get();
        return  ( (elevatorEncoder.getVelocity() > 0.01) && (this.getEncoderInches() > kForwardSoftLimit)   );
    }

    /** Return true when the elevator is at the bottom. */
    public boolean isAtBot() {
 //       return m_ElevatorAtBot.get();
        return  ( (elevatorEncoder.getVelocity() < -0.01) && (this.getEncoderInches() < kReverseSoftLimit)   );
    }




    public void log() {
        SmartDashboard.putNumber("Elevator Encoder inches", getEncoderInches());
        SmartDashboard.putNumber("Elevator Lt motor temperature", getLeftElevatorTemp());
        SmartDashboard.putNumber("Elevator Rt motor temperature", getRightElevatorTemp());

      }
     /** Get Temp. */
      public double getLeftElevatorTemp() {
      return leftElevatorMotor.getMotorTemperature();
   }
      public double getRightElevatorTemp() {
         return rightElevatorMotor.getMotorTemperature();
      }

}






