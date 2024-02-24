package frc.robot.commands;


import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElbowSubsystem;
//import frc.robot.ConstantsMechanisms.ElbowConstants;
//import frc.robot.ConstantsMechanisms.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// Position Arm to a given Elbow and Elevator Position 
// Start Elevator first to clear robot frame before extending elevator
public class ArmPosition extends SequentialCommandGroup {
  
  public ArmPosition(ElevatorSubsystem elevator, ElbowSubsystem elbow , double elevatorPosition, double elbowAngle) {

    addCommands(
      Commands.parallel(
        new ElbowPIDCmd(elbow, elbowAngle, 0.0),  

        Commands.sequence(
          new WaitCommand(0.25),
          new ElevatorPIDCmd(elevator, elevatorPosition,  0.0)     
        ) 
      )
    );
  }
}

    

