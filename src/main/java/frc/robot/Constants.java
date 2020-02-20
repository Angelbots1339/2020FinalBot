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
    }

    public final class IntakeConstants{
        // SparkMAX Motors
        public final static int kIntakeMotor = 5;
        public final static int kLeftIntakeMoverMotor = 9;
        public final static int kRightIntakeMoverMotor = 8;

        public final static double kMaxIntakeSpeed = 0.5; // speed
        public final static double kMaxIntakeMoverSpeed = 0.5;
    }

    public final class IndexerConstants{
        // SparkMAX Motors
        public final static int kLeftIndexerMotor = 7;
        public final static int kRightIndexMotor = 6;

        public final static double kMaxIndexSpeed = 0.5; // speed

    }

    public final class LoaderConstants{
        public final static int kLoaderMotor = 2; // SparkMAX
        public final static double kMaxLoaderSpeed = 0.5; // speed
    }

    public final static class ShooterConstants{
        public final static int kLeftShooter = 1;
        public final static int kRightShooter = 3;

        // PID values
        // TODO have to be tuned to current robot values
        public final static double kP = 1.4;
        public final static double kI = 0.01;
        public final static double kD = 0;
        public final static double kff = 0;

        // setting up encoder
        public final static int[] kEncoderPorts = new int[]{4, 5}; //TODO
        public final static boolean kEnocderReversed = true;
        public final static int kEncoderCPR = 1024; // clicks per rotation

        // distance units int rotations
        public final static double kEncoderDistancePerPulse = 1.0 / (double) kEncoderCPR;

        public final static double kShooterFreeRPS = 5300; 
        public final static double kShooterTargetRPS = 4000; // rotations per second
        public final static double kShooterToleranceRPS = 50;

        // TODO need to be empirically determined, these are reasonable guesses
        public final static double KSVolts = 0.05;
        // should have value 12V at free speed...
        public final static double KVVoltSecondsPerRotation = 12.0 / kShooterFreeRPS;
        public final static double kSpeed = 0.5;

        public final static double kMaxShooterSpeed = 0.5;

    }
        
    public final static class ClimberConstants{
        public final static int kLeftClimberMotor = 10;
        public final static int kRightClimberMotor = 11;
        public final static int kLeftServo = -1;
        public final static int kRightServo = -1;

        public final static int[] kLeftEncoder = new int[]{1,2}; //TODO find the ports
        public final static int[] kRightEncoder = new int[]{3, 4};
        public final static boolean kLeftEncoderReversed = false; //TODO 
        public final static boolean krightEncoderReversed = false; //TODOs
        public final static double kClimberSpeed = 0.0;

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
    }

    public final class BuddyClimbConstants{
        public final static int kServoPort = 0;
        public final static double kServoValue = 0;
    }

    public final class HoodedShooterConstants{
        public final static int kHoodPort = 4;

        public final static double kP = 1.4;
        public final static double kI = 0.01;
        public final static double kD = 0;

        public final static double positionTolerance = 0.075;
    }

    public final class SensorConstants{
        // Beam Brakes
        public final static int topReciever = 0;
        public final static int topEmitter = 1;
        public final static int middleReciever = 2;
        public final static int middleEmitter = 3;
        public final static int bottomReciever = 4;
        public final static int bottomEmitter = 5;
    }

    public final class OIconstants{
         // Main Joystick USB Port
         public static final int kDriverControllerPort = 1;
         public static final int kOperatorControllerPort = 0;
         public static final int kTestControllerPort = 2;
         
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
