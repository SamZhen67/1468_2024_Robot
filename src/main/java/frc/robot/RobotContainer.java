package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/* // For choreo path generation
import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory; */


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /*A chooser for autonomous commands */
    SendableChooser<Command> autonomousChooser = new SendableChooser<>();
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton centerAprilTag = new JoystickButton(driver, PS4Controller.Button.kCircle.value);


    /* Operator Buttons  */
  
    private final JoystickButton shootButton = new JoystickButton(operator, 1);
    private final JoystickButton harvestButton = new JoystickButton(operator, 3);
    private final JoystickButton ejectButton = new JoystickButton(operator, 4);

    private final JoystickButton elbowUpButton = new JoystickButton(operator, 9);
    private final JoystickButton elbowDownButton = new JoystickButton(operator, 10);
    private final JoystickButton elevatorUpButton = new JoystickButton(operator, 11);
    private final JoystickButton elevatorDownButton = new JoystickButton(operator, 12);
    /* temporary buttons  */
    private final JoystickButton climberUpButton = new JoystickButton(operator,5);
    private final JoystickButton climberDownButton = new JoystickButton(operator, 6);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight();
    private final HarvesterSubsystem s_Harvester = new HarvesterSubsystem();
    private final StorageSubsystem s_Storage = new StorageSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final ElbowSubsystem s_Elbow = new ElbowSubsystem();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final ClimberSubsystem s_Climber = new ClimberSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        // Build auto chooser
        autonomousChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Choose Auto", autonomousChooser);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        centerAprilTag.whileTrue(Commands.parallel(new CenterAprilTagPrinting(s_Limelight), new SlideToAprilTag(s_Swerve)));
 
        /* Operator Buttons */
        shootButton.debounce(.1).whileTrue(new ShootNote(s_Shooter, s_Storage));
        harvestButton.debounce(.1).whileTrue(new HarvestNote(s_Harvester, s_Storage));
        ejectButton.debounce(.1).whileTrue(new EjectNote(s_Harvester, s_Storage));
  
        elbowUpButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Elbow.elbowUp()));
        elbowDownButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Elbow.elbowDown()));
        elevatorUpButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorUp()));
        elevatorDownButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorDown()));
        climberUpButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Climber.climberUp()));
        climberDownButton.debounce(.1).whileTrue(new InstantCommand(() -> s_Climber.climberDown()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autonomousChooser.getSelected();
    }
}