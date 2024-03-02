// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import static frc.robot.ConstantsMechanisms.ShooterConstants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  
  // Shooter Motor Controllers
  private TalonFX m_leftShooterMotor = new TalonFX(SHOOTER_LEFT_MOTOR_ID,"rio"); // KRAKEN motor
//  private RelativeEncoder shooterEncoder =  m_leftShooterMotor.getEncoder();//
  private TalonFX m_rightShooterMotor = new TalonFX(SHOOTER_RIGHT_MOTOR_ID,"rio"); // KRAKEN motor



/* Be able to switch which control request to use based on a button press */
  /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  /* Start at velocity 0, no feed forward, use slot 1 */
  private final VelocityTorqueCurrentFOC m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, false, false, false);
  /* Keep a neutral out so we can disable the motor */
  private final NeutralOut m_brake = new NeutralOut();

//  private final Mechanisms m_mechanisms = new Mechanisms();




    /** Subsystem for controlling the Shooter */
  public ShooterSubsystem() {
    m_leftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_leftShooterMotor.setInverted(false);            // TA TODO: polarity of these? - was true
    m_rightShooterMotor.setInverted(false);
     





    TalonFXConfiguration configs = new TalonFXConfiguration();

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;
    
    /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
    configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
    configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output

    // Peak output of 40 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_leftShooterMotor.getConfigurator().apply(configs);
      status = m_rightShooterMotor.getConfigurator().apply(configs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }





  }

  /* Set  Shooter motors to Brake Mode */
  public void setBrakeMode() {
    m_leftShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    m_rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /* Set  Shooter motors to Brake Mode */
  public void setCoastMode() {
    m_leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    m_rightShooterMotor.setNeutralMode(NeutralModeValue.Coast);
  }


  /* Set power to the Shooter motor */
  public void setShooterSpeeds(double ltSpeed, double rtSpeed) {
    m_leftShooterMotor.set(-ltSpeed);
    m_rightShooterMotor.set(rtSpeed);  
  }


  /* Set power to the Shooter motor */
  public void setShooterVoltageVelos(double ltSpeed, double rtSpeed) {

    m_leftShooterMotor.setControl(m_voltageVelocity.withVelocity(-ltSpeed * 50));
    m_rightShooterMotor.setControl(m_voltageVelocity.withVelocity(rtSpeed * 50)); 
  }

  /* Set power to the Shooter motor */
  public void setShooterTorqueVelos(double ltSpeed, double rtSpeed) {
    m_leftShooterMotor.setControl(m_torqueVelocity.withVelocity(-ltSpeed * 50).withFeedForward(-1));  // 1 didnt work

    m_rightShooterMotor.setControl(m_torqueVelocity.withVelocity(rtSpeed * 50).withFeedForward(-1));

  }




  /* Set power to the Shooter motor */
 // public void shootNote() {
 //   m_leftShooterMotor.set(SHOOT_SPEED);
//  }

  /* Set power to the Shooter motor */
  public void ejectNote() {
    m_leftShooterMotor.set(-SHOOTER_EJECT_SPEED);
    m_rightShooterMotor.set(SHOOTER_EJECT_SPEED);  
  }

  public void stop() {
    m_leftShooterMotor.set(0);
    m_rightShooterMotor.set(0);
   }

  @Override
  public void periodic() {
    
    // Put the speed on SmartDashboard
    SmartDashboard.putNumber("Shooter LtMtr Speed", m_leftShooterMotor.get());
    SmartDashboard.putNumber("Shooter RtMtr Speed", m_rightShooterMotor.get());
    SmartDashboard.putNumber("Shooter LtMtr Velo", m_leftShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter RtMtr Velo", m_rightShooterMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Lt motor temperature", m_leftShooterMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Rt motor temperature", m_rightShooterMotor.getDeviceTemp().getValueAsDouble());

  }
}