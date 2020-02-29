/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    public final class DriveConstants{
        // TalonFX Motors
        public final static int kLeftFrontMotor = 13; // 3
        public final static int kLeftBackMotor = 14; // 4
        public final static int kRightFrontMotor = 12; // 2
        public final static int kRightBackMotor = 15; // 1

        public final static double kMaxDriveSpeed = 0.5; // speed

        public final static double kMinPower = 0.2; // speed
    }

    public final class IntakeConstants{
        // SparkMAX Motors
        public final static int kIntakeMotor = 5;
        public final static int kLeftIntakeMoverMotor = 9; // not inverted in fireware
        public final static int kRightIntakeMoverMotor = 8; // inverted in firmware

        public final static double kMaxIntakeSpeed = 1; // speed was 0.45 , set slower for testing
        public final static double kMaxIntakeArmSpeed = 0.15; // was mixed up in the intake class

        public final static double kMaxNormalVoltage = 3; // needed to be a bit lower 5;
        public final static double kmaxVeloValue = 0.3;
        public final static double kmaxNormalCurrent = 12;

        // Intake Arm Constants
        public final static double kP = 0.9;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kIntakeArmMotorVolt = 6;
        public final static double kminEncoderValue = -10;
        public final static double kmaxEncoderValue = 0;
        public final static double positionTolerance = 1;
    }

    public final class IndexerConstants{
        // SparkMAX Motors
        public final static int kLeftIndexerMotor = 7;
        public final static int kRightIndexMotor = 6;
        // Speed was at 0.45 for Scrimmage
        public final static double kMaxIndexSpeed = 0.7; // speed, was 0.5
        public final static double kAcceptableCurrentSpike = 0;

    }

    public final class LoaderConstants{
        public final static int kLoaderMotor = 2; // SparkMAX
        // Changed by Ender, was 0.55 for Scrimmage
        // This speed is used when feeding balls to shooter
        public final static double kMaxLoaderSpeed = 0.8; // speed
        // This speed is used when loading first ball
        // public final static double kInitLoaderSpeed = 0.2; // speed
        public final static double kInitLoaderSpeed = 0.6; // speed
    }

    public final static class ShooterConstants{
        public final static int kLeftShooter = 1;
        public final static int kRightShooter = 3;

        // PID values
        // TODO have to be tuned to current robot values
        public final static double kP = 0.001; //0.001;//0.0008;
        public final static double kI = 0.00002;
        public final static double kD = 0;
        public final static double kff = 0;

        // setting up encoder
        public final static int[] kEncoderPorts = new int[]{4, 5}; //TODO
        public final static boolean kEnocderReversed = true;
        public final static int kEncoderCPR = 4096; // clicks per rotation

        // distance units int rotations
        public final static double kEncoderDistancePerPulse = 1.0 / (double) kEncoderCPR;
        public final static double kShooterFreeRPS = 5676; 
        public final static double kShooterTargetRPS = 5450; // ~5500 is max speed, rotations per second, 3 to 2 gear ratio  
        public final static double kShooterToleranceRPS = 35;

        // 0.18 to 0.23 seems to be a sweet spot, varies based on Battery?
        public final static double KSVolts = 0.23; // ~0.23 was found empirically, working well
        public final static double KVVoltSecondsPerRotation = 12 / kShooterFreeRPS;  // 12 V is working well
        public final static double kMaxShooterSpeed = 0.5;

    }
        
    public final static class ClimberConstants{
        public final static int kLeftClimberMotor = 10;
        public final static int kRightClimberMotor = 11;
        public final static int kLeftServo = 0; // TODO double check
        public final static int kRightServo = 9; // TODO doulbe check

        public final static int[] kLeftEncoder = new int[]{1,2}; //TODO find the ports
        public final static int[] kRightEncoder = new int[]{3, 4};
        public final static boolean kLeftEncoderReversed = false; //TODO 
        public final static boolean krightEncoderReversed = false; //TODOs

        public final static double kClimberSpeed = 0.7;

        public final static double kDownP = 0;
        public final static double kDownI = 0;
        public final static double kDownD = 0;
        public final static double kDownFF = 0;

        public final static double kUpP = 0;
        public final static double kUpI = 0;
        public final static double kUpD = 0;
        public final static double kUpFF = 0;
        
        public final static double kSVolts = 0;
        public final static double kSVoltsPerSecondsPerRotation = 0;

        public final static double kClimberToleranceRPS = 0;
        public final static double kEncoderDistancePerPulse = 0;
        public final static double kClimberTargetRPS = 0;

        public final static double degrees = 25;
    }

    public final class BuddyClimbConstants{
        public final static int kServoPort = 0;
        public final static double kServoValue = 0;
    }

    public final class HoodedShooterConstants{
        public final static int kHoodPort = 4;

        public final static double kmaxVeloValue = 0.2;
        public final static double kmaxNormalCurrent = 18;
        // PID HOOD
        public final static double kP = 0.8;
        public final static double kI = 0.0;
        public final static double kD = 0;
        public final static double kMaxHoodVolt = 4;
        public final static double kminEncoderValue = 0;
        public final static double kmaxEncoderValue = 17.5;
        public final static double positionTolerance = .35;
		public static final double KSVolts = 0.13;
        public static final double KVVoltSecondsPerRotation = 12 * (12.5 / 11000); // Neo 550 - 11000 rpm, with 12.5:1 reduction
        
    }
    public static final class LimelightConstants {
        // Ensure that the limelight hostname is just "limelight", otherwise there will be nothing
        // on networktable for data.  Check network table data in Shuffleboard when troubleshooting
        public final static String kLimeTable = "limelight";
        public final static int kDefaultPipeline = 0;
        public static final double kAngleP = 0.03;
        public static final double kDriveP = 0.25;
        public static final double kDriveTolerance = 0.095;//used to be 0.055
        public static final double kLimelightHeight = 0.43;
        public static final double kTagetHeight = 2.08;
        public static final double kLimelightToTargetHeight = kTagetHeight - kLimelightHeight;
        public static final double kVerticalFOV = 49.7;
        public static final double kHorizontalFOV = 59.6;
        public static final double kLimeLightTilt = 25;
        public static final double k2XZoomCutoff = 3.785;//TODO
        public static final double k3XZoomCutoff = 5.436;//TODO
        public static final boolean kAutoZoom = true;
    

        public final static double minEncoderValue = 0.1;
        public final static double maxEncoderValue = 17.5;
        public final static double maxVeloValue = 0.2;
        public final static double maxNormalCurrent = 18;

        public final static double positionTolerance = 0.075;

		public static final int kMaxHoodVolt = 7;
    }

    public final class SensorConstants{

		public final static int topEmitter = 5; //1; broke them so we had to switch the ports
		public final static int topReciever = 4; //0;
		public final static int middleEmitter = 3;
		public final static int middleReciever = 2;
		public final static int ShooterEmitter = 0; //5; // right next to shooter, counts when balls leave
		public final static int ShooterReciever = 1; // 4;
		public final static int bottomEmitter = 6; // counts when balls enter
		public final static int bottomReciever = 7;

    }

    public final class OIconstants{

         // Main Joystick USB Port
         public static final int kDriverControllerPort = 1;
         public static final int kOperatorControllerPort = 0;
         public static final int kTestControllerPort = 2;
         
         public static final double kLeftTriggerThreshold = 0.5;
         public static final double kRightTriggerThreshold = 0.5;
         
         public final static int leftYAxis = 1;      // speed 
         public final static int leftXAxis = 0;      //
         public final static int rightYAxis = 5;     //
         public final static int rightXAxis = 4;     // turn
         public final static int bButton = 2;        // shooter
         public final static int yButton = 4;        // reverse indexer
         public final static int xButton = 3;        // forward indexer
         public final static int aButton = 1;        // 
         public final static int leftBumper = 5;     // reverse intake
         public final static int rightBumper = 6;    // forward intake
         public final static int startButton = 8;    //
         public final static int backButton = 7;     //
         public final static int leftButton = 9;     //
         public final static int rightButton = 10;   //

    }

    

}
