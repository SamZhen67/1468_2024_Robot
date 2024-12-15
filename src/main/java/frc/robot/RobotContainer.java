package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.PS4Controller;
//import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.ConstantsMechanisms.*;

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
    final Swerve s_Swerve = new Swerve();
    final Limelight s_Limelight = new Limelight();
    final HarvesterSubsystem s_Harvester = new HarvesterSubsystem();
    final StorageSubsystem s_Storage = new StorageSubsystem();
    final ShooterSubsystem s_Shooter = new ShooterSubsystem();
    public  ElbowSubsystem s_Elbow = new ElbowSubsystem();
//    final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
    final ClimberSubsystem s_Climber = new ClimberSubsystem();
    final BlinkinLEDController s_LedSubsystem = new BlinkinLEDController();

      
    
    // Driver Using Joysticks
    final Joystick driverLeftJoystick = new Joystick(0);
    final Joystick driverRightJoystick = new Joystick(1);

    final Joystick operatorJoystick = new Joystick(3);  
    final Joystick testOprJoystick = new Joystick(2);  // this was for testing purposes only
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Register named commands for PathPlanner Auto Routines

        NamedCommands.registerCommand("Home", 
            new InstantCommand(() -> s_Shooter.stop())
            .alongWith(new ElbowPIDCmd( s_Elbow, ElbowConstants.kHomeAngle, ElbowConstants.kTolerance )));

        NamedCommands.registerCommand("Harvest", 
            new HarvestNote(s_Harvester, s_Storage, s_LedSubsystem).withTimeout(3.5));          // TODO: TA - might want to remove timeout here and put in Pathplanner - different times for differnet distances

        NamedCommands.registerCommand("Shoot", 
            new ShootNote( s_Storage,s_LedSubsystem));        

        NamedCommands.registerCommand("AimSubWoofer", 
            new ElbowPIDCmd( s_Elbow, (ElbowConstants.kScoreInSpeakerFromSubwooferAngle+1), ElbowConstants.kAutoTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(75,50 )))); 

        NamedCommands.registerCommand("HoldSubWoofer", 
            new ElbowPIDCmd( s_Elbow, (ElbowConstants.kScoreInSpeakerFromSubwooferAngle+1), ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(75,50 )))); 
 
            NamedCommands.registerCommand("AimPodium", 
            new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInSpeakerFromPodiumAngle, ElbowConstants.kAutoTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 

        NamedCommands.registerCommand("HoldPodium", 
            new ElbowPIDCmd( s_Elbow,  ElbowConstants.kScoreInSpeakerFromPodiumAngle, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 

            NamedCommands.registerCommand("AimFar", 
            new ElbowPIDCmd( s_Elbow, 37.5, ElbowConstants.kAutoTolerance )                             // was 35.5
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 

        NamedCommands.registerCommand("HoldFar", 
            new ElbowPIDCmd( s_Elbow,  37.5, ElbowConstants.kHoldStillTolerance )                          // was 35.5
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 

 
        // Show what command your subsystem is running on the SmartDashboard
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(s_Limelight);
        SmartDashboard.putData(s_Harvester);
        SmartDashboard.putData(s_Storage);
        SmartDashboard.putData(s_Shooter);
        SmartDashboard.putData(s_Elbow);
//        SmartDashboard.putData(s_Elevator);
        SmartDashboard.putData(s_Climber);

        // Configure the button bindings
        configureDriverBindings();
        configureOperatorBindings();

        autonomousConfig();

        brakeOrCoastModeConfig();

        // Put Some buttons on the SmartDashboard
//        SmartDashboard.putData("PitchDrve", new ChargeStationBalanceDriveYaw0(s_Swerve));

    }


    private void configureDriverBindings() {

        // Driver Buttons
        final JoystickButton lt1RobotCentric = new JoystickButton(driverLeftJoystick, 1);
        final JoystickButton lt5FastSpin = new JoystickButton(driverLeftJoystick, 5);
        final JoystickButton lt3CenterAmpAprilTag = new JoystickButton(driverLeftJoystick, 4);
        final JoystickButton lt4CenterTrapAprilTag = new JoystickButton(driverLeftJoystick, 5);

        final JoystickButton driveZeroGyro = new JoystickButton(driverRightJoystick, 1);
        final JoystickButton zeroGyro = new JoystickButton(driverRightJoystick, 7);
        final JoystickButton resetPose = new JoystickButton(driverRightJoystick, 8);
        final JoystickButton zeroArm = new JoystickButton(driverRightJoystick, 9);
        final JoystickButton rt4CenterAmpAprilTag = new JoystickButton(driverRightJoystick, 4);
        final JoystickButton rt3RotateSpeakerAprilTag = new JoystickButton(driverRightJoystick, 3);
        final JoystickButton rt5CenterTrapAprilTag = new JoystickButton(driverRightJoystick, 5);
        final JoystickButton centerNote = new JoystickButton(driverRightJoystick, 11);


        // Assign default swerve command
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX()/ 1.5,            // slow down rotate - TA TODO - fix the way Im doing this 
                () -> lt1RobotCentric.getAsBoolean()
            )
        );

        lt5FastSpin.debounce(.1).whileTrue(            
            new TeleopSwerve(
                s_Swerve, 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(),             
                () -> lt1RobotCentric.getAsBoolean()
            )
        );




        // Connect the driver buttons to commands

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        resetPose.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(0,0),  Rotation2d.fromDegrees(0)))));
//        zeroArm.onTrue(new InstantCommand(() -> s_Elbow.resetElbowEncGyro()).alongWith(new InstantCommand(() -> s_Elevator.resetElevatorEncGyro())));
        zeroArm.onTrue(new InstantCommand(() -> s_Elbow.resetElbowEncGyro()));

        driveZeroGyro.debounce(.1).whileTrue(            
            new TeleopSwerveZeroYaw(
                s_Swerve, 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(),             
                () -> lt1RobotCentric.getAsBoolean()
            )
        );

        rt3RotateSpeakerAprilTag.whileTrue( new TeleopSwerveRotatetoSpeakerAprilTag(s_Swerve,                 
                () -> -driverLeftJoystick.getY(), 
                () -> -driverLeftJoystick.getX(), 
                () -> -driverRightJoystick.getX(),            
                () -> lt1RobotCentric.getAsBoolean()));

        centerNote.whileTrue(new DriveToNote(
            s_Swerve, 0));
        centerNote.onFalse(new InstantCommand( () -> s_Swerve.stop( )));
        
        rt4CenterAmpAprilTag.whileTrue( new DriveToAprilTag(s_Swerve, LimelightConstants.AMP_PIPELINE, ConstantsMechanisms.kAmpShotDistanceFromAprilTag,s_LedSubsystem));
        rt4CenterAmpAprilTag.onFalse(new InstantCommand( () -> s_Swerve.stop( )));
      
        rt5CenterTrapAprilTag.whileTrue( new DriveToAprilTag(s_Swerve, LimelightConstants.TRAP_PIPELINE, ConstantsMechanisms.kTrapShotDistanceFromAprilTag,s_LedSubsystem));
        rt5CenterTrapAprilTag.onFalse(new InstantCommand( () -> s_Swerve.stop( )));
        
      
        lt3CenterAmpAprilTag.whileTrue( new DriveToAprilTagPP(s_Swerve, LimelightConstants.AMP_PIPELINE, ConstantsMechanisms.kAmpShotDistanceFromAprilTag));
        lt3CenterAmpAprilTag.onFalse(new InstantCommand( () -> s_Swerve.stop( )));
        lt4CenterTrapAprilTag.whileTrue( new DriveToAprilTagPP(s_Swerve, LimelightConstants.TRAP_PIPELINE, ConstantsMechanisms.kTrapShotDistanceFromAprilTag));
        lt4CenterTrapAprilTag.onFalse(new InstantCommand( () -> s_Swerve.stop( )));
  

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
//        final JoystickButton b5prepareToShootSpeakerButton = new JoystickButton(operatorJoystick, 5);
        final JoystickButton b5PassButton = new JoystickButton(operatorJoystick, 5);
        final JoystickButton b6prepareToShootTrapButton = new JoystickButton(operatorJoystick, 6);
// remove - not working        
//        final JoystickButton b7smallUpELbowButton = new JoystickButton(operatorJoystick, 7);
//        final JoystickButton b8smallDownElbowButton = new JoystickButton(operatorJoystick, 8);

//        final JoystickButton b9smallUpELevButton = new JoystickButton(operatorJoystick, 9);
//        final JoystickButton b10smallDownELevButton = new JoystickButton(operatorJoystick, 10);


        final JoystickButton b11climberUpButton = new JoystickButton(operatorJoystick, 11);
        final JoystickButton b12climberDownButton = new JoystickButton(operatorJoystick, 12);

        final POVButton UpPovArmToPodiumShot = new POVButton(operatorJoystick, 0);
        final POVButton downPovArmToSubwooferShot = new POVButton(operatorJoystick, 180);

        final POVButton rightPovArmTo4MShot = new POVButton(operatorJoystick, 90);
        final POVButton leftPovArmTo4pt5MShot = new POVButton(operatorJoystick, 270);


        b1shootButton.debounce(.1).whileTrue(new ShootNote(s_Storage,s_LedSubsystem));
        b1shootButton.debounce(.1).onFalse((new InstantCommand(() -> s_Storage.stop())
            .alongWith(new InstantCommand(() -> s_Shooter.stop()))
            .alongWith(new ElbowPIDCmd( s_Elbow, ElbowConstants.kHomeAngle, ElbowConstants.kTolerance ))));
           
        b2harvestButton.debounce(.1)
            .whileTrue(new HarvestNote(s_Harvester, s_Storage, s_LedSubsystem));

        b3prepareToHarvestButton.debounce(.1)
            .onTrue( new ElbowPIDCmd(s_Elbow, ElbowConstants.kHomeAngle, ElbowConstants.kTolerance ) 
            .alongWith(new InstantCommand(() -> s_Shooter.stop())));
 
        b4prepareToShootAmpButton.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, 50, ElbowConstants.kHoldStillTolerance ));
        b4prepareToShootAmpButton.debounce(.1)
            .onFalse(new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInAmpAngle, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand(() -> s_Shooter.setShooterVoltageVelos(20,20 )))); 


        b5PassButton.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInSpeakerFromSubwooferAngle, ElbowConstants.kHoldStillTolerance )    //was kScoreInTrapAngle
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(60,40 ))));       //was 50,30


        UpPovArmToPodiumShot.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInSpeakerFromPodiumAngle, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 
        downPovArmToSubwooferShot.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInSpeakerFromSubwooferAngle, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 


        rightPovArmTo4MShot.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, 35.5, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 
        leftPovArmTo4pt5MShot.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, 39, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand( () -> s_Shooter.setShooterVoltageVelos(90,60 )))); 



        b6prepareToShootTrapButton.debounce(.1)
            .onTrue(new ElbowPIDCmd( s_Elbow, ElbowConstants.kScoreInTrapAngle, ElbowConstants.kHoldStillTolerance )
            .alongWith(new InstantCommand(() -> s_Shooter.setShooterVoltageVelos(64,41 )))); // maybe 45,41
//            .alongWith(new SetShooterVelocity(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ(),  false)));
 
        // +/-180 are the indicators for doing small up/down moves
//        b7smallUpELbowButton.debounce(.1).onTrue(new ElbowPIDCmd(s_Elbow, +180, 0.0 ));       
//        b8smallDownElbowButton.debounce(.1).onTrue(new ElbowPIDCmd(s_Elbow, -180, 0.0 ));     

// couldnt get PID to work, use velocity mode instead
//        b11climberUpButton.debounce(.1)
//            .onTrue(new ClimberPIDCmd(s_Climber, ClimberConstants.kMaxVelUp, ClimberConstants.kMaxAccUp, ClimberConstants.kClimbPosition));
//        b12climberDownButton.debounce(.1)
//            .onTrue(new ClimberPIDCmd(s_Climber, ClimberConstants.kMaxVelDown, ClimberConstants.kMaxAccDown, ClimberConstants.kHomePosition));

        b11climberUpButton.debounce(.1)
            .onTrue(new ClimberVeloCmd(s_Climber, ClimberConstants.kUpSpeed));
        b11climberUpButton.debounce(.1)
            .onFalse(new ClimberVeloCmd(s_Climber, 0.0));
        b12climberDownButton.debounce(.1)
            .onTrue(new ClimberVeloCmd(s_Climber,ClimberConstants.kDownSpeed));
        b12climberDownButton.debounce(.1)
            .onFalse(new ClimberVeloCmd(s_Climber, 0.0));
  
/* */
//Test joystick, buttons and bindings

        final JoystickButton t1 = new JoystickButton(testOprJoystick, 1);
        final JoystickButton t2 = new JoystickButton(testOprJoystick, 2);
        final JoystickButton t3 = new JoystickButton(testOprJoystick,3);
//        final JoystickButton t4 = new JoystickButton(testOprJoystick,4);
//        final JoystickButton t5 = new JoystickButton(testOprJoystick, 5);
        final JoystickButton t6 = new JoystickButton(testOprJoystick, 6);
        final JoystickButton t7 = new JoystickButton(testOprJoystick, 7);
        final JoystickButton t8 = new JoystickButton(testOprJoystick, 8);
        final JoystickButton t9 = new JoystickButton(testOprJoystick, 9);
        final JoystickButton t10 = new JoystickButton(testOprJoystick, 10);
        final JoystickButton t11 = new JoystickButton(testOprJoystick, 11);
//        final JoystickButton t12 = new JoystickButton(testOprJoystick, 12);       // no button 12 on this JyStk this year


        t1.debounce(.1)
            .whileTrue(new EjectNote(s_Harvester, s_Storage, s_Shooter));
        t2.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Climber.climberDown()));
        t2.debounce(.1)
            .onFalse(new InstantCommand(() -> s_Climber.climberStop()));
        t3.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Climber.climberUp()));
        t3.debounce(.1)
            .onFalse(new InstantCommand(() -> s_Climber.climberStop()));


        t6.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Elbow.elbowUp()));
        t6.debounce(.1)
            .onFalse(new InstantCommand(() -> s_Elbow.elbowStop()));
        t7.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Elbow.elbowDown()));
        t7.debounce(.1)
            .onFalse(new InstantCommand(() -> s_Elbow.elbowStop()));
         t8.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Storage.getNote()));
        t8.debounce(.1)
            .onFalse(new InstantCommand(() -> s_Storage.stop()));
        t9.debounce(.1)
            .whileTrue(new InstantCommand(() -> s_Shooter.stop()));


//        t10.debounce(.1)
//            .whileTrue(new SetShooterVelocity(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ(),  true));
        t10.debounce(.1)
            .onFalse(new InstantCommand(() ->  s_Shooter.stop()));
        t11.debounce(.1)
            .whileTrue(new SetShooterVelocity(s_Shooter, () -> driverRightJoystick.getZ(), () -> testOprJoystick.getZ(),  false));
        t11.debounce(.1)
            .onFalse(new InstantCommand(() ->  s_Shooter.stop()));


//        t10.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorDown()));
//        t10.debounce(.1).onFalse(new InstantCommand(() -> s_Elevator.elevatorStop()));
//        t11.debounce(.1).whileTrue(new InstantCommand(() -> s_Elevator.elevatorUp()));
//        t11.debounce(.1).onFalse(new InstantCommand(() -> s_Elevator.elevatorStop()));


// End of Test Button Logic       */


 
 
 
    }


    private void autonomousConfig() {
        
        // Build auto chooser
//        autonomousChooser = AutoBuilder.buildAutoChooser();
        autonomousChooser = AutoBuilder.buildAutoChooser("Lt4Note");

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



 

    private void brakeOrCoastModeConfig() {
 /*  
// Turn brake mode off shortly after the robot is disabled
SwerveModule mod;
new Trigger(this::isEnabled) // Create a trigger that is active when the robot is enabled
    .negate() // Negate the trigger, so it is active when the robot is disabled
    .debounce(3) // Delay action until robot has been disabled for a certain time
    .onTrue( // Finally take action
        new InstantCommand( // Instant command will execute our "initialize" method and finish immediately
            () -> mod.setBrakeMode(), // Enable coast mode in drive train
            s_Swerve) // command requires subsystem
            .ignoringDisable(true)); // This command can run when the robot is disabled


   
        for(SwerveModule mod : s_Swerve.mSwerveMods){
            mod. setBrakeMode();
        }

   */
   
       

    }





}