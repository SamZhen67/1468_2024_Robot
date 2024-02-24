package frc.robot.commands;

import frc.robot.subsystems.StorageSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootNote extends SequentialCommandGroup {
 
  public ShootNote(  StorageSubsystem storage ) {

    addCommands(
      new InstantCommand(() -> storage.getNote()),
      new WaitCommand(2.025),
      new InstantCommand(() -> storage.stop())
    );
  }
}

