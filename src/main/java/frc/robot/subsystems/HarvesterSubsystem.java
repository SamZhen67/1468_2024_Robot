// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.HarvesterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HarvesterSubsystem extends SubsystemBase {
  
  // Harvester Motor Controller
  private CANSparkMax m_harvester; // NEO motor

  // Harvester Limit Switch
  DigitalInput harvesterLimitSwitch = new DigitalInput(HARVESTER_LIMIT_SWITCH_ID);

  /** Subsystem for controlling the harvester */
  public HarvesterSubsystem() {
    // Instantiate the harvester motor controller
    m_harvester = new CANSparkMax(HARVESTER_MOTOR_ID, MotorType.kBrushless);
    m_harvester.setSmartCurrentLimit(60);
    m_harvester.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Reverse it if needed
    m_harvester.setInverted(HARVESTER_INVERT);
  }

  /* Set harvester motor to Brake Mode */
  public void setBrakeMode() {
     m_harvester.setIdleMode(CANSparkMax.IdleMode.kBrake);
 }

  /* Set harvester motor to Coast Mode */
  public void setCoastMode() {
     m_harvester.setIdleMode(CANSparkMax.IdleMode.kCoast);
 }


  /* Set power to the harvester motor */
  public void getNote() {
    m_harvester.set(HARVESTER_IN_SPEED);
  }
/* Set power to the harvester motor */
  public void ejectNote() {
    m_harvester.set(HARVESTER_EJECT_SPEED);
  }

  public void stop() {
    m_harvester.set(0);
  }

  public boolean getHarvesterLimitSwitch() {
    return harvesterLimitSwitch.get();
  }

  @Override
  public void periodic() {
    
    // Put the speed on SmartDashboard
    SmartDashboard.putNumber("Harvester Speed", m_harvester.get());
    SmartDashboard.putBoolean("Harvester Limit Switch", getHarvesterLimitSwitch());
    SmartDashboard.putNumber("Harvester Motor Temp", m_harvester.getMotorTemperature());

  }
}