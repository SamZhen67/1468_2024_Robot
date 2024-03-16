package frc.robot.commands;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
//import static frc.robot.ConstantsMechanisms.ClimberConstants.*;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberVeloCmd extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private double speed;

    public ClimberVeloCmd(ClimberSubsystem climberSubsystem, Double speed) {
        m_climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override   
    public void initialize() {
        m_climberSubsystem.setMotorSpeed(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

        m_climberSubsystem.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return (m_climberSubsystem.isAtBot()||m_climberSubsystem.isAtTop());        
    }
}
