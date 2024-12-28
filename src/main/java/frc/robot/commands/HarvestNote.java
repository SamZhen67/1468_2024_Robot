// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BlinkinLEDController;
import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class HarvestNote extends Command {
private final HarvesterSubsystem m_harvester;
private final StorageSubsystem m_storage ;
private BlinkinLEDController m_ledCont;

  public HarvestNote(HarvesterSubsystem harvester, StorageSubsystem storage, BlinkinLEDController ledCont ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_harvester = harvester; 
    addRequirements(m_harvester);
    m_storage = storage;
    addRequirements(m_storage);
        m_ledCont = ledCont;
    addRequirements(m_ledCont);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_harvester.getNote();
    m_storage.getNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_storage.getStorageLimitSwitch())     m_ledCont.LED_Harvesting();
    else m_ledCont.LED_Harvested();

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_harvester.stop();
    m_storage.stop();
   // m_ledCont.off();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

//    return false;   // TODO - FIX WHEN LIMIT SW IS INSTALLED
    return m_storage.getStorageLimitSwitch();
  }
}
