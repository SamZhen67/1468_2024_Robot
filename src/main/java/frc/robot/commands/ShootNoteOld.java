package frc.robot.commands;

import frc.robot.subsystems.StorageSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootNoteOld extends SequentialCommandGroup {
 
  public ShootNoteOld(  StorageSubsystem storage ) {

    addCommands(
      new InstantCommand(() -> storage.getNote()),
      new WaitCommand(1.525)     ,                     // .5 too short
      new InstantCommand(() -> storage.stop())
    );
  }
}

