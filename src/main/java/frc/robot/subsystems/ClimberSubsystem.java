   // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.ClimberConstants.*;

//import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax leftClimberMotor = new CANSparkMax(kLeftClimberMotorPort, CANSparkMax.MotorType.kBrushless);
    private RelativeEncoder climberEncoder =  leftClimberMotor.getEncoder();//
    private CANSparkMax rightClimberMotor = new CANSparkMax(kRightClimberMotorPort, CANSparkMax.MotorType.kBrushless);
 
    public ClimberSubsystem() {
        leftClimberMotor.setInverted(false);            // TA TODO: polarity of these?
        rightClimberMotor.setInverted(true);
        rightClimberMotor.follow(leftClimberMotor, true);    // TA TODO: check if both inverts are needed
        
        climberEncoder.setPosition(kZeroOffset);     // TA TODO: Determine offset
        climberEncoder.setPositionConversionFactor(kEncoderRotation2Inches); // TA TODO: Determine conversion factor
        leftClimberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, kForwardSoftLimit);
        leftClimberMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, kReverseSoftLimit);
        leftClimberMotor.setSmartCurrentLimit(60);
        rightClimberMotor.setSmartCurrentLimit(60);
        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      }

    @Override
    public void periodic() {
       log();
    }

    public void setCoastMode() {
      leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setBrakeMode() {
      leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
      rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
  
    public SparkPIDController getIntegratedSparkPID( ) {
        return leftClimberMotor.getPIDController();
     }
    
    
    public void setMotorSpeed(double speed) {
       leftClimberMotor.set(speed);
    }


    public void climberUp( ) {
       leftClimberMotor.set(kUpSpeed);
    }

 
     public void climberDown() {
        leftClimberMotor.set(kDownSpeed);
     }

  
    public void climberStop() {
        leftClimberMotor.set(0);
    }


    public double getEncoderInches() {
      return climberEncoder.getPosition() ;
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

    /** Return true when the climber is at the top. (no hard stop on top)*/
    public boolean isAtTop() {
        return  ( (leftClimberMotor.get() > 0.1) && (this.getEncoderInches() > kForwardSoftLimit)   );
    }

    /** Return true when the climber is at the bottom. (give 1/8" margin is its a hard stop) */
    public boolean isAtBot() {
        return  ( (leftClimberMotor.get() < -0.1) && (this.getEncoderInches() < (kReverseSoftLimit+.125))   );
    }




    public void log() {
        SmartDashboard.putNumber("Climber Encoder inches", getEncoderInches());
        SmartDashboard.putNumber("Climber Speed", leftClimberMotor.get());
        SmartDashboard.putNumber("Climber Lt motor temperature", getLeftClimberTemp());
        SmartDashboard.putNumber("Climber Rt motor temperature", getRightClimberTemp());

      }
     /** Get Temp. */
      public double getLeftClimberTemp() {
      return leftClimberMotor.getMotorTemperature();
   }
      public double getRightClimberTemp() {
         return rightClimberMotor.getMotorTemperature();
      }

}






