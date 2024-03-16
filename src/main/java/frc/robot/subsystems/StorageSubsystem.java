// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.StorageConstants.*;

//import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StorageSubsystem extends SubsystemBase {
  
  // Storage Motor Controller
 // private CANSparkMax m_storage; // NEO motor - swapped out
  // Shooter Motor Controllers
  private TalonFX m_storage = new TalonFX(STORAGE_MOTOR_ID,"rio"); // KRAKEN motor

    // Storage Limit Switch
  DigitalInput storageLimitSwitch = new DigitalInput(STORAGE_LIMIT_SWITCH_ID);

  /** Subsystem for controlling the Storage */
  public StorageSubsystem() {
    // Instantiate the Storage motor controller
//    m_storage = new CANSparkMax(STORAGE_MOTOR_ID, MotorType.kBrushless);

    m_storage.setNeutralMode(NeutralModeValue.Brake);

    // Reverse it if needed
    m_storage.setInverted(STORAGE_INVERT);
  }

  /* Set Storage motor to Brake Mode */
  public void setBrakeMode() {
  m_storage.setNeutralMode(NeutralModeValue.Brake);
  }
  /* Set Storage motor to Brake Mode */
  public void setCoastMode() {
  m_storage.setNeutralMode(NeutralModeValue.Coast);
  }

  /* Set power to the Storage motor */
  public void getNote() {
    m_storage.set(STORAGE_IN_SPEED);
  }
/* Set power to the Storage motor */
  public void ejectNote() {
    m_storage.set(STORAGE_EJECT_SPEED);
  }

  public void stop() {
    m_storage.set(0);
  }

  public boolean getStorageLimitSwitch() {
    return storageLimitSwitch.get();
  }

  @Override
  public void periodic() {
    
    // Put the speed on SmartDashboard
    SmartDashboard.putNumber("Storage Mtr Speed", m_storage.get());
    SmartDashboard.putBoolean("Storage Limit Switch", getStorageLimitSwitch());
    SmartDashboard.putNumber("Storage Motor Temp", m_storage.getDeviceTemp().getValueAsDouble());

  }
}