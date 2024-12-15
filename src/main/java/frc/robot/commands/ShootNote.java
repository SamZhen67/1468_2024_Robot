// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BlinkinLEDController;

import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


public class ShootNote extends Command {
private final StorageSubsystem m_storage ;
private BlinkinLEDController m_ledCont;
private double startTime, delayStopTime, currentTime;
private int falseCoutner = 0;         // the limit switch should go false twice when we shoot a note out 
private boolean currentState, previousState, done;

  public ShootNote( StorageSubsystem storage,  BlinkinLEDController ledCont) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_storage = storage;
    addRequirements(m_storage);
    m_ledCont = ledCont;
    addRequirements(m_ledCont);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    startTime = Timer.getFPGATimestamp();
    falseCoutner = 0;
    currentState = m_storage.getStorageLimitSwitch();
    previousState = currentState;


    m_storage.getNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentState = m_storage.getStorageLimitSwitch();
    if (!previousState && currentState) {
      falseCoutner++;
      if (falseCoutner == 1)     delayStopTime = Timer.getFPGATimestamp();
    }  

    currentTime = Timer.getFPGATimestamp();
    if (currentTime - startTime > 1.15) done = true; // saftey check, should never take more than 1 second
    if ((falseCoutner >= 1) && (currentTime - delayStopTime > .250)) done = true; // dont stop harvestor until we're sure the note is gone - give an extra 250 msec to be sure.

    if (!done) m_ledCont.LED_Shooting();
    else m_ledCont.off();

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

//    return false;   // TODO - FIX WHEN LIMIT SW IS INSTALLED
    return done;
  }
}
