package frc.robot.commands;


import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;

import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ArmHome extends SequentialCommandGroup {
 
  public ArmHome(ElevatorSubsystem elevator, ElbowSubsystem elbow ) {
    addCommands(
       Commands.parallel(
        new ElevatorPIDCmd(elevator,  ElevatorConstants.kHomePosition, ElevatorConstants.kTolerance),    

          Commands.sequence(
            new WaitCommand(0.55),
            new ElbowPIDCmd(elbow, ElbowConstants.kHomeAngle, ElbowConstants.kTolerance) 
          )
        ) 

    
         
    );
  }
}

    

