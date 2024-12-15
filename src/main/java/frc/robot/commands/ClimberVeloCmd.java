package frc.robot.commands;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.ConstantsMechanisms.ClimberConstants.*;
import frc.robot.subsystems.ClimberSubsystem;


public class ClimberVeloCmd extends Command {
    private final ClimberSubsystem m_climberSubsystem;
    private double m_speed;

    public ClimberVeloCmd(ClimberSubsystem climberSubsystem, Double speed) {
        m_climberSubsystem = climberSubsystem;
        m_speed = speed;
        addRequirements(climberSubsystem);
    }

    @Override   
    public void initialize() {
        m_climberSubsystem.setMotorSpeed(0.0);
    }

    @Override
    public void execute() {

        if ( (m_speed > 0.0) && (m_climberSubsystem.getEncoderInches() < kForwardSoftLimit)) m_climberSubsystem.setMotorSpeed(m_speed);
        else if ( (m_speed < 0.0) && (m_climberSubsystem.getEncoderInches() > kReverseStop)) m_climberSubsystem.setMotorSpeed(m_speed);
        else  m_climberSubsystem.setMotorSpeed(0.0);    // stop motor if it is going down and at bottom

    }

    @Override
    public void end(boolean interrupted) {

        m_climberSubsystem.setMotorSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return  (m_speed == 0.0);        
    }
}
