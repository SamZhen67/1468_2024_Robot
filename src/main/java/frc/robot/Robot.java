// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.net.PortForwarder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final CTREConfigs ctreConfigs = new CTREConfigs();

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double startTime, currentTime;
  private boolean disabledCoastModeSet = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();


    // Setup Port Forwarding to enable Limelight communication while tethered to your robot over USB.
    // Forward ports 5800, 5801, 5802, 5803, 5804, 5805, 5806, and 5807
    for (int port = 5800; port <= 5807; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

    startTime = Timer.getFPGATimestamp();
    disabledCoastModeSet = false;

  }

  @Override
  public void disabledPeriodic() {
/**/
    currentTime =  Timer.getFPGATimestamp();
    if( (currentTime-startTime > 7.0) && !disabledCoastModeSet) {
      m_robotContainer.s_Climber.setCoastMode();
      m_robotContainer.s_Elbow.setCoastMode();
//      m_robotContainer.s_Elevator.setCoastMode();
      m_robotContainer.s_Harvester.setCoastMode();
      m_robotContainer.s_Shooter.setCoastMode();
      m_robotContainer.s_Storage.setCoastMode();
      m_robotContainer.s_Swerve.mSwerveMods[0].setCoastMode();
      m_robotContainer.s_Swerve.mSwerveMods[1].setCoastMode();
      m_robotContainer.s_Swerve.mSwerveMods[2].setCoastMode();
      m_robotContainer.s_Swerve.mSwerveMods[3].setCoastMode();

      disabledCoastModeSet = true;
    }

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
 /*   */
      m_robotContainer.s_Climber.setBrakeMode();
      m_robotContainer.s_Elbow.setBrakeMode();
//      m_robotContainer.s_Elevator.setBrakeMode();
      m_robotContainer.s_Harvester.setBrakeMode();
//      m_robotContainer.s_Shooter.setBrakeMode();      // TODO: TA - try coast mode 
      m_robotContainer.s_Storage.setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[0].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[1].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[2].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[3].setBrakeMode();
//*/
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
/* */
      m_robotContainer.s_Climber.setBrakeMode();
      m_robotContainer.s_Elbow.setBrakeMode();
//      m_robotContainer.s_Elevator.setBrakeMode();
      m_robotContainer.s_Harvester.setBrakeMode();
//      m_robotContainer.s_Shooter.setBrakeMode();      // TODO: TA - try coast mode 
      m_robotContainer.s_Storage.setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[0].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[1].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[2].setBrakeMode();
      m_robotContainer.s_Swerve.mSwerveMods[3].setBrakeMode();
//*/
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}