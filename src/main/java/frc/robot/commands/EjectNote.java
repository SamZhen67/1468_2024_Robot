// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.HarvesterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class EjectNote extends Command {
private final HarvesterSubsystem m_harvester;
private final StorageSubsystem m_storage;
  public EjectNote(HarvesterSubsystem harvester, StorageSubsystem storage) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_harvester = harvester;
    addRequirements(m_harvester);
    m_storage = storage;
    addRequirements(m_storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_harvester.ejectNote();
    m_storage.ejectNote();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_harvester.stop();
    m_storage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //  return !m_harvester.getHarvesterLimitSwitch();
 return false;
}
}
