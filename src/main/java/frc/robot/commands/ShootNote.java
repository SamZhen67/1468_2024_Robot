package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootNote extends SequentialCommandGroup {
 
  public ShootNote( ShooterSubsystem shooter, StorageSubsystem storage ) {
    addCommands(
        new InstantCommand(() -> shooter.shootNote()), 
        new InstantCommand(() -> storage.ejectNote()).withTimeout(.25), 
        new WaitCommand(0.75),
        new InstantCommand(() -> storage.getNote())   

    );
  }
}

