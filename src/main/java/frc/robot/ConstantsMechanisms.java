// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

//import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class ConstantsMechanisms {


//////////////////////////    HARVESTER     /////////////////////////////////////////
 
public static final class HarvesterConstants {
    public static final int HARVESTER_MOTOR_ID = 21;  
    public static final boolean HARVESTER_INVERT = false;
    public static final int HARVESTER_MOTOR_PDH_CHANNEL = 5; // TA TODO: check placement
    public static final double HARVESTER_IN_SPEED = 0.5;
    public static final double HARVESTER_EJECT_SPEED = -0.95;
    public static final int HARVESTER_LIMIT_SWITCH_ID = 1;  
}   
 

//////////////////////////    STORAGE     /////////////////////////////////////////

    public static final class StorageConstants {
    public static final int STORAGE_MOTOR_ID = 22;  
    public static final boolean STORAGE_INVERT = false;
    public static final int STORAGE_MOTOR_PDH_CHANNEL = 5; // TA TODO: check placement
    public static final double STORAGE_IN_SPEED = 0.5;
    public static final double STORAGE_EJECT_SPEED = -0.95;
    public static final int STORAGE_LIMIT_SWITCH_ID = 2;  
}   


//////////////////////////    SHOOTER     /////////////////////////////////////////

    public static final class ShooterConstants {
    public static final int SHOOTER_LEFT_MOTOR_ID = 23;  
    public static final int SHOOTER_RIGHT_MOTOR_ID = 24;  
    public static final boolean SHOOTER_INVERT = false;
    public static final int SHOOTER_MOTOR_PDH_CHANNEL = 5; // TA TODO: check placement
    public static final double SHOOT_SPEED = 0.95;
    public static final double SHOOTER_EJECT_SPEED = -0.95;
    public static final int SHOOTER_LIMIT_SWITCH_ID = 2;  
}   


//////////////////////////    ELBOW     /////////////////////////////////////////

public static final class ElbowConstants {
    public static final int kLeftElbowMotorPort = 31;
    public static final int kRightElbowMotorPort = 32;
    

    // there are 4096 ticks per revolution - this is the denominator -  CanSParkMax uses rotations not ticks
    // numerator is conversion to degrees taking into account the gear ratio (125:1 - was 100:1) and sprocket ratio (52:28)
//        public static final double kEncoderTick2Degrees = 1.0 / 4096.0 *  360 * 100 * 52/28;
//      public static final double kEncoderTick2Degrees = 1.0 / 4096.0 *  360 / 100 * 28/52;
    public static final double kEncoderRotation2Degrees = 1.0  *  360 / 125 * 28/52  * 1.3444444;

    // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
    // ****    This is a Position based PID That uses a Trapazoidal Profile for movement
    public static final double kP = 5e-5; 
    public static final double kI = 1e-6;   //TODO: I think this is correct
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.000156; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
    //       public static final double maxRPM = 5700;
    // Smart Motion Coefficients
    public static final int smartMotionSlot = 0;
    public static final double kMinVel = 0;      // rpm
    public static final double kMaxVel = 3000;   // rpm
    public static final double kMaxAcc = 2000;      // TODO: TA - Optimize Max Vel and Acc (Acc was 3000 - seemed to fast)
    public static final double kAllowedErr = 0;

// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;



    public static final double kZeroOffset = -27;
    public static final double kStartAngle = -22;
    public static final double kRetreiveFromFloorAngle = 175 ;
    public static final double kRetreiveFromFloorAngleAuto = 97 ;      //105 too low
    public static final double kRetreiveFromShelfAngle = -2.5; // 0 was too far out , -10 was too far in      
    public static final double kRetreiveFromShelfFarAngle = +5.5; //9.5 was too far out    
//        public static final double kPlaceOnRow1Angle = 165;
    public static final double kPlaceOnRow2Angle = 27.5 ;      // was 25, tried 20, but not out far enough
    public static final double kPlaceOnRow2AngleAuto = 31 ;      // was  33 try to stuff cone on in auto.
    public static final double kPlaceOnRow3Angle = 90 ;    

    public static float kReverseSoftLimit = -22;
    public static float kForwardSoftLimit = 180;

    public static float kSmallMoveDegrees = 5;
    public static final double kMaxAngle = (kForwardSoftLimit-3);


    public static final double kUpSpeed = 0.2;
    public static final double kDownSpeed = -0.2;

    public static final double kTolerance = 1.50;           // was 0.5 - to small


}



//////////////////////////    ELEVATOR     /////////////////////////////////////////
    
public static final class ElevatorConstants {
    public static final int kLeftElMotorPort = 33;
    public static final int kRightElMotorPort = 34;

    public static final double kUpSpeed = 0.15;      // 0.5 too fast for now
    public static final double kDownSpeed = -0.05;

    // CanSparkMax Encoder native units is rotations. We convert to inches for the elevator
    // rotations * circumference of wheel (pi*d) / the gear ratio: 
    // d= sprocket diameter * pulley diameter, 1.757*1.041" and gear down is 5:1 
    // THere is a factor of 2 that I dont understand?!?
    public static final double kEncoderRotation2Inches = 1.0 * Math.PI * (1.757 * 1.041) / 5  * 1.925061;

    // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
// ****    This is a Position based PID That uses a Trapazoidal Profile for movement
    public static final double kP = 0.00020; 
    public static final double kI = 0.0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
//       public static final double maxRPM = 5700;
// Smart Motion Coefficients
    public static final int smartMotionSlot = 0;
    public static final double kMinVel = 0;      // rpm
    public static final double kMaxVel = 4000;   // rpm
    public static final double kMaxAcc = 4000;
    public static final double kAllowedErr = 0;
    
// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;


    public static final double kZeroOffset = 0.0;
    public static final double kHomePosition = 0.25 ;       // Keep home very slightly above hard stop
    public static final double kRow2Height = 4.5 ;      //was 7
    public static final double kRow3Height = 45 ;       // was 45, but wanted higeer since elbow drifts down quickly
    public static final double kFloorHeight = 25 ;
    public static final double kShelfHeight = 4.55 ;       //was 5
    public static final double kShelfFarHeight = 5.0 ;       // was 5.5

//        public static final double kTolerance = 0.75 ;      // was 0.25 - to small
    public static final double kTolerance = 1.25 ;      // was 0.75 - need bigger now that we have wear and ear on elevator
    
    public static float kReverseSoftLimit = -1;
    public static float kForwardSoftLimit = 48;

    public static double kSmallMoveInches = 3;

    

    public static final double kJoystickMaxSpeed = 0.5 ;
}




//////////////////////////    CLIMBER     /////////////////////////////////////////
    
public static final class ClimberConstants {
    public static final int kLeftClimerMotorPort = 41;
    public static final int kRightClimberMotorPort = 42;

    public static final double kUpSpeed = 0.15;      // 0.5 too fast for now
    public static final double kDownSpeed = -0.05;

    // CanSparkMax Encoder native units is rotations. We convert to inches for the elevator
    // rotations * circumference of wheel (pi*d) / the gear ratio: 
    // d= sprocket diameter * pulley diameter, 1.757*1.041" and gear down is 5:1 
    // THere is a factor of 2 that I dont understand?!?
    public static final double kEncoderRotation2Inches = 1.0 * Math.PI * (1.757 * 1.041) / 5  * 1.925061;

    // ****    Below are the PID values for Smart Motion Mode PID - DO not remove;
// ****    This is a Position based PID That uses a Trapazoidal Profile for movement
    public static final double kP = 0.00020; 
    public static final double kI = 0.0;
    public static final double kD = 0; 
    public static final double kIz = 0; 
    public static final double kFF = 0.0; 
    public static final double kMaxOutput = 1; 
    public static final double kMinOutput = -1;
//       public static final double maxRPM = 5700;
// Smart Motion Coefficients
    public static final int smartMotionSlot = 0;
    public static final double kMinVel = 0;      // rpm
    public static final double kMaxVel = 4000;   // rpm
    public static final double kMaxAcc = 4000;
    public static final double kAllowedErr = 0;
    
// ****    Above are the PID values for Smart Motion Mode PID - DO not remove;


    public static final double kZeroOffset = 0.0;
    public static final double kHomePosition = 0.25 ;       // Keep home very slightly above hard stop
    public static final double kRow2Height = 4.5 ;      //was 7
    public static final double kRow3Height = 45 ;       // was 45, but wanted higeer since elbow drifts down quickly
    public static final double kFloorHeight = 25 ;
    public static final double kShelfHeight = 4.55 ;       //was 5
    public static final double kShelfFarHeight = 5.0 ;       // was 5.5

//        public static final double kTolerance = 0.75 ;      // was 0.25 - to small
    public static final double kTolerance = 1.25 ;      // was 0.75 - need bigger now that we have wear and ear on elevator
    
    public static float kReverseSoftLimit = -1;
    public static float kForwardSoftLimit = 48;

    public static double kSmallMoveInches = 3;

    

    public static final double kJoystickMaxSpeed = 0.5 ;
}



/*   
    // Apriltag Vision Constants //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);
    public static final String USB_CAMERA_NAME = "USB_Camera-B4.09.24.1";

*/
}