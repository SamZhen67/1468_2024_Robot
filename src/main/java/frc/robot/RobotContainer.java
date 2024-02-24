package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ConstantsMechanisms.ClimberConstants;
import frc.robot.ConstantsMechanisms.ElbowConstants;
import frc.robot.ConstantsMechanisms.ElevatorConstants;
import frc.robot.commands.*;

import frc.robot.subsystems.*;

//import static frc.robot.ConstantsMechanisms.ElbowConstants;
     

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


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight();
    private final HarvesterSubsystem s_Harvester = new HarvesterSubsystem();
    private final StorageSubsystem s_Storage = new StorageSubsystem();
    private final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    private final ElbowSubsystem s_Elbow = new ElbowSubsystem();
    private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    private final ClimberSubsystem s_Climber = new ClimberSubsystem();

      
    
    // Driver Using Joysticks
    final Joystick driverLeftJoystick = new Joystick(0);
    final Joystick driverRightJoystick = new Joystick(1);

    final Joystick operatorJoystick = new Joystick(3);  
    final Joystick testOprJoystick = new Joystick(2);  // this was for testing purposes only
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Show what command your subsystem is running on the SmartDashboard
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(s_Limelight);
        SmartDashboard.putData(s_Harvester);
        SmartDashboard.putData(s_Storage);
        SmartDashboard.putData(s_Shooter);
        SmartDashboard.putData(s_Elbow);
        SmartDashboard.putData(s_Elevator);
        SmartDashboard.putData(s_Climber);

        // Configure the button bindings
        configureDriverBindings();
        configureOperatorBindings();

        autonomousConfig();
  
        // Put Some buttons on the SmartDashboard
//        SmartDashboard.putData("PitchDrve", new ChargeStationBalanceDriveYaw0(s_Swerve));

    }


    private void configureDriverBindings() {

        // Driver Buttons
        final JoystickButton robotCentric = new JoystickButton(driverLeftJoystick, 1);

        final JoystickButton centerAprilTag = new JoystickButton(driverLeftJoystick, 3);
//        final JoystickButton slideLeft = new JoystickButton(driverLeftJoystick, 11);
//        final JoystickButton slideRight = new JoystickButton(driverLeftJoystick, 12); 

        final JoystickButton driveZeroGyro = new JoystickButton(driverRightJoystick, 1);
        final JoystickButton zeroGyro = new JoystickButton(driverRightJoystick, 7);
        final JoystickButton resetPose = new JoystickButton(driverRightJoystick, 8);
        final JoystickButton zeroArm = new JoystickButton(driverRightJoystick, 9);

        // Connect the buttons to commands
        // Assign default swerve command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(), 
                () -> robotCentric.getAsBoolean()
            )
        );

        driveZeroGyro.debounce(.1).whileTrue(            
            new TeleopSwerveZeroYaw(
                s_Swerve, 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(), 
                () -> robotCentric.getAsBoolean()
            )
        );

       /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        centerAprilTag.whileTrue(Commands.parallel(new CenterAprilTagPrinting(s_Limelight), new SlideToAprilTag(s_Swerve)));
        resetPose.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0,0),  Rotation2d.fromDegrees(0)))));
        zeroArm.onTrue(new InstantCommand(() -> s_Elbow.resetElbowEncGyro()).alongWith(new InstantCommand(() -> s_Elevator.resetElevatorEncGyro())));

//        slideLeft.debounce(.1).onTrue(new SlideLeft(s_Swerve ));
//        slideRight.debounce(.1).onTrue(new SlideRight(s_Swerve ));

  


/*         // Driver Using PS4

     private final Joystick driverPS4Cont = new Joystick(0);
        // Driver Buttons
        final int translationAxis = PS4Controller.Axis.kLeftY.value;
        final int strafeAxis = PS4Controller.Axis.kLeftX.value;
        final int rotationAxis = PS4Controller.Axis.kRightX.value;
        final JoystickButton robotCentric = new JoystickButton(driverPS4Cont, PS4Controller.Button.kL1.value);

        final JoystickButton zeroGyro = new JoystickButton(driverPS4Cont, PS4Controller.Button.kTriangle.value);
        final JoystickButton slideLeft = new JoystickButton(driverPS4Cont, PS4Controller.Button.kL1.value);
        final JoystickButton slideRight = new JoystickButton(driverPS4Cont, PS4Controller.Button.kR1.value);

        // Connect the buttons to commands
        // Assign default swerve command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
 //               () -> -driverPS4Cont.getRawAxis(translationAxis),         //was inverted?!?
 //               () -> -driverPS4Cont.getRawAxis(strafeAxis), 
                () -> driverPS4Cont.getRawAxis(translationAxis), 
                () -> driverPS4Cont.getRawAxis(strafeAxis), 
                () -> -driverPS4Cont.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        slideLeft.debounce(.1).onTrue(new SlideLeft(s_Swerve ));
        slideRight.debounce(.1).onTrue(new SlideRight(s_Swerve ));

*/

    }


    private void configureOperatorBindings() {

        final JoystickButton b1shootButton = new JoystickButton(operatorJoystick, 1);
        final JoystickButton b2harvestButton = new JoystickButton(operatorJoystick, 2);

        final JoystickButton b3prepareToHarvestButton = new JoystickButton(operatorJoystick, 3);
        final JoystickButton b4prepareToShootAmpButton = new JoystickButton(operatorJoystick, 4);
        final JoystickButton b5prepareToShootSpeakerButton = new JoystickButton(operatorJoystick, 5);
        final JoystickButton b6prepareToShootTrapButton = new JoystickButton(operatorJoystick, 6);
        
        final JoystickButton b7smallUpELbowButton = new JoystickButton(operatorJoystick, 7);
        final JoystickButton b8smallDownElbowButton = new JoystickButton(operatorJoystick, 8);
        final JoystickButton b9smallUpELevButton = new JoystickButton(operatorJoystick, 9);
        final JoystickButton b10smallDownELevButton = new JoystickButton(operatorJoystick, 10);
        final JoystickButton b11climberUpButton = new JoystickButton(operatorJoystick, 11);
        final JoystickButton b12climberDownButton = new JoystickButton(operatorJoystick, 12);

        b1shootButton.debounce(.1).onTrue(new ShootNote( s_Storage));
           
//        b2harvestButton.debounce(.1).onTrue(new HarvestNote(s_Harvester, s_Storage));  // TODO FIX WHEN LIMIT SW IS INSTALLED
        b2harvestButton.debounce(.1).whileTrue(new HarvestNote(s_Harvester, s_Storage));

        b3prepareToHarvestButton.debounce(.1).onTrue(new ArmHome(s_Elevator, s_Elbow)
                                        .alongWith(new InstantCommand(() -> s_Shooter.stop())));
        b4prepareToShootAmpButton.debounce(.1).onTrue(
            new ArmPosition(s_Elevator, s_Elbow, ElevatorConstants.kScoreInAmpPosition, ElbowConstants.kScoreInAmpAngle).
            alongWith(new InstantCommand(() -> s_Shooter.setShooterSpeeds (.2, .2)))); 
        b5prepareToShootSpeakerButton.debounce(.1).onTrue(
//            new PositionElbowForSpeakerShot(s_Elbow).
            new ElbowPIDCmdAT(s_Elbow,ElbowConstants.kScoreInSpeakerFromPodiumAngle,0.0).
            alongWith(new SetShooterSpeed(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ())));
        b6prepareToShootTrapButton.debounce(.1).onTrue(
            new ArmPosition(s_Elevator, s_Elbow, ElevatorConstants.kScoreInTrapPosition, ElbowConstants.kScoreInTrapAngle).
            alongWith(new SetShooterSpeed(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ())));
 
        // +/-180 are the indicators for doing small up/down moves
        b7smallUpELbowButton.debounce(.1).onTrue(new ElbowPIDCmd(s_Elbow, +180, 0.0));       
        b8smallDownElbowButton.debounce(.1).onTrue(new ElbowPIDCmd(s_Elbow, -180, 0.0));     
        b9smallUpELevButton.debounce(.1).onTrue(new ElevatorPIDCmd(s_Elevator, +180, 0.0));  
        b10smallDownELevButton.debounce(.1).onTrue(new ElevatorPIDCmd(s_Elevator, -180, 0.0));  

        b11climberUpButton.debounce(.1).onTrue(new ClimberPIDCmd(s_Climber, ClimberConstants.kMaxVelUp, ClimberConstants.kMaxAccUp, ClimberConstants.kClimbPosition));
        b12climberDownButton.debounce(.1).onTrue(new ClimberPIDCmd(s_Climber, ClimberConstants.kMaxVelDown, ClimberConstants.kMaxAccDown, ClimberConstants.kHomePosition));
  
  
/* */
//Test joystick, buttons and bindings

        final JoystickButton t1 = new JoystickButton(testOprJoystick, 1);
        final JoystickButton t2 = new JoystickButton(testOprJoystick, 2);
        final JoystickButton t3 = new JoystickButton(testOprJoystick,3);
        final JoystickButton t4 = new JoystickButton(testOprJoystick,4);
        final JoystickButton t5 = new JoystickButton(testOprJoystick, 5);
        final JoystickButton t6 = new JoystickButton(testOprJoystick, 6);
        final JoystickButton t7 = new JoystickButton(testOprJoystick, 7);
        final JoystickButton t8 = new JoystickButton(testOprJoystick, 8);
        final JoystickButton t9 = new JoystickButton(testOprJoystick, 9);
        final JoystickButton t10 = new JoystickButton(testOprJoystick, 10);
        final JoystickButton t11 = new JoystickButton(testOprJoystick, 11);
//        final JoystickButton t12 = new JoystickButton(testOprJoystick, 12);       // no button 12 on this JyStk this year


        t1.debounce(.1).whileTrue(new EjectNote(s_Harvester, s_Storage, s_Shooter));

        t2.debounce(.1).whileTrue(new InstantCommand(() -> s_Climber.climberDown()));
        t2.debounce(.1).onFalse(new InstantCommand(() -> s_Climber.climberStop()));
        t3.debounce(.1).whileTrue(new InstantCommand(() -> s_Climber.climberUp()));
        t3.debounce(.1).onFalse(new InstantCommand(() -> s_Climber.climberStop()));

        t4.debounce(.1).onTrue( new ArmPosition(s_Elevator, s_Elbow, ElevatorConstants.kHomePosition, ElbowConstants.kScoreInSpeakerFromSubwooferAngle).
            alongWith(new InstantCommand(() -> s_Shooter.setShooterSpeeds (.75, .5))));       
        t5.debounce(.1).onTrue( new ArmPosition(s_Elevator, s_Elbow, ElevatorConstants.kHomePosition, ElbowConstants.kScoreInSpeakerFromPodiumAngle).
            alongWith(new SetShooterSpeed(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ())));       

        t6.debounce(.1).whileTrue(new InstantCommand(() -> s_Elbow.elbowUp()));
        t6.debounce(.1).onFalse(new InstantCommand(() -> s_Elbow.elbowStop()));
        t7.debounce(.1).whileTrue(new InstantCommand(() -> s_Elbow.elbowDown()));
        t7.debounce(.1).onFalse(new InstantCommand(() -> s_Elbow.elbowStop()));
 
        t8.debounce(.1).whileTrue(new InstantCommand(() -> s_Storage.getNote()));
        t8.debounce(.1).onFalse(new InstantCommand(() -> s_Storage.stop()));
 
        t9.debounce(.1).whileTrue(new InstantCommand(() -> s_Shooter.stop()));

        t10.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorDown()));
        t10.debounce(.1).onFalse(new InstantCommand(() -> s_Elevator.elevatorStop()));
        t11.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorUp()));
        t11.debounce(.1).onFalse(new InstantCommand(() -> s_Elevator.elevatorStop()));


// End of Test Button Logic       */


 
 
 
    }




    private void autonomousConfig() {
        
        // Build auto chooser
//        autonomousChooser = AutoBuilder.buildAutoChooser();
        autonomousChooser = AutoBuilder.buildAutoChooser("1ft");

       // Put the chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autonomousChooser);
//       SmartDashboard.putData(autonomousChooser);   
    }


    // Use this to pass the autonomous command to the main {@link Robot} class.
    // @return the command to run in autonomous  
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autonomousChooser.getSelected();
    }
}