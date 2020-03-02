/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class DriveConstants {
        // TalonFX Motors
        public final static int kLeftFrontMotor = 13;
        public final static int kLeftBackMotor = 14;
        public final static int kRightFrontMotor = 12;
        public final static int kRightBackMotor = 15;

        public final static double kMaxDriveSpeed = 0.6; // speed

        public final static double kMinPower = 0.2; // speed
        public final static double kTurnInPlaceThreshold = 0.2;
    }

    public final class IntakeConstants {
        public final static int kIntakeMotor = 5;
        public final static int kLeftIntakeMoverMotor = 9;  // not inverted in fireware
        public final static int kRightIntakeMoverMotor = 8; // inverted in firmware

        public final static double kMaxIntakeSpeed = 1;    
        public final static double kMaxIntakeArmSpeed = 0.15;

        public final static double kMaxNormalVoltage = 3;  
        public final static double kMaxVeloValue = 0.3;
        public final static double kMaxNormalCurrent = 12;

        // Intake Arm Constants
        public final static double kP = 0.4;//45; //98;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kIntakeArmMotorVolt = 5;
        public final static double kMinEncoderValue = -10;
        public final static double kMaxEncoderValue = -2;
        public final static double kPositionTolerance = 0.2;
    }
    public final class IndexerConstants{
        public final static int kLeftIndexerMotor = 7;
        public final static int kRightIndexMotor = 6;
        public final static double kMaxIndexSpeed = 0.7;
        public final static double kAcceptableCurrentSpike = 0;

    }

    public final class LoaderConstants {
        public final static int kLoaderMotor = 2; 
        // This speed is used when feeding balls to shooter
        public final static double kFeedLoaderSpeed = 0.6; // speed
        // This speed is used when loading first ball
        public final static double kInitLoaderSpeed = 0.5; // speed
        // This speed is used when reversing
        public final static double kReverseLoaderSpeed = 0.6; // speed
		public final static double kP = 0.001;
		public final static double kI = 0;
		public final static double kD = 0;
        public final static double kLoaderToleranceRPS = 30;
        public final static double kLoaderSetpoint = 30;

    }

    public final static class ShooterConstants {
        public final static int kLeftShooter = 1;
        public final static int kRightShooter = 3;

        // PID values
        // TODO have to be tuned to current robot values
        public final static double kP = 0.0008; // 0.002, 0.001;//0.0008;
        public final static double kI = 0.00001;
        public final static double kD = 0;
        public final static double kFF = 0;

        // setting up encoder
        //public final static int[] kEncoderPorts = new int[] { 4, 5 }; // TODO
        //public final static boolean kEnocderReversed = true;
        public final static int kEncoderCPR = 4096; // clicks per rotation

        // distance units int rotations
        public final static double kEncoderDistancePerPulse = 1.0 / (double) kEncoderCPR;
        public final static double kShooterFreeRPS = 5676;
        public final static double kShooterTargetRPM = 5450; // ~5500 is max speed, rotations per second, 3 to 2 gear ratio
        public final static double kShooterToleranceRPS = 22;

        // 0.18 to 0.23 seems to be a sweet spot, varies based on Battery?
        public final static double KSVolts = 0.23; // ~0.23 was found empirically, working well
        public final static double KVVoltSecondsPerRotation = 12 / kShooterFreeRPS; // 12 V is working well
        public final static double kMaxShooterSpeed = 0.5;

    }

    public final class HoodConstants {
        public final static int kHoodPort = 4;

        public final static double kMaxVeloValue = 0.2;
        public final static double kMaxNormalCurrent = 18;
        // PID HOOD
        public final static double kP = 0.9;    //0.8
        public final static double kI = 0.0;
        public final static double kD = 0.019;
        public final static double kMaxHoodVolt = 1.5;
        public final static double kMinEncoderValue = 0;
        public final static double kmaxEncoderValue = 17.5;

        public final static double kMinResistedVoltage = 60;
        public final static double kPositionTolerance = .35;
		public final static double KSVolts = 0.13;
        public final static double KVVoltSecondsPerRotation = 12 * (12.5 / 11000); // Neo 550 - 11000 rpm, with 12.5:1 reduction
    }

    public final static class ClimberConstants {
        public final static int kLeftClimberMotor = 10;
        public final static int kRightClimberMotor = 11;
        public final static int kLeftServo = 0;     // TODO double check
        public final static int kRightServo = 9;    // TODO doulbe check

        public final static int[] kLeftEncoder = new int[] { 1, 2 }; // TODO find the ports
        public final static int[] kRightEncoder = new int[] { 3, 4 };
        public final static boolean kLeftEncoderReversed = false;   // TODO
        public final static boolean kRightEncoderReversed = false;  // TODOs

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
    }

    public final class BuddyClimbConstants {
        public final static int kServoPort = 0;
        public final static double kServoValue = 0;
    }

    public final static class LimelightConstants {
        // Ensure that the limelight hostname is just "limelight", otherwise there will
        // be nothing on networktable for data. Check network table data in Shuffleboard when
        // troubleshooting
        public final static String kLimeTable = "limelight";
        public final static int kDefaultPipeline = 0;
        public final static double kDriveP = 0.25;
        public final static double kDriveTolerance = 0.4;// used to be 0.055
        public final static double kLimelightHeight = 0.43;
        public final static double kTagetHeight = 2.08;
        public final static double kLimelightToTargetHeight = kTagetHeight - kLimelightHeight;
        public final static double kVerticalFOV = 49.7;
        public final static double kHorizontalFOV = 59.6;
        public final static double kLimeLightTilt = 25;
        public final static double k2XZoomCutoff = 3.785;// TODO
        public final static double k3XZoomCutoff = 5.436;// TODO
        public final static double[] kPanningOffest = { 0, -9, -6 };// TODO
        public final static boolean kAutoZoom = true;

        public final static double kMinEncoderValue = 0.1;
        public final static double kMaxEncoderValue = 17.5;
        public final static double kMaxVeloValue = 0.2;
        public final static double kMaxNormalCurrent = 18;

        public final static double kPositionTolerance = 0.075;

        public final static int kMaxHoodVolt = 7;
    }

    public final class SensorConstants {

        public final static int topEmitter = 5;     // 1; broke them so we had to switch the ports
        public final static int topReciever = 4;    // 0;
        public final static int middleEmitter = 3;
        public final static int middleReciever = 2;
        public final static int ShooterEmitter = 0; // 5; // right next to shooter, counts when balls leave
        public final static int ShooterReciever = 1;// 4;
        public final static int bottomEmitter = 6;  // counts when balls enter
        public final static int bottomReciever = 7;

    }

    public final class OIconstants {

        // Main Joystick USB Port
        public final static int kDriverControllerPort = 0;
        public final static int kTestControllerPort = 2;
        
        public final static double kLeftTriggerThreshold = 0.5;
        public final static double kRightTriggerThreshold = 0.5;
    }

    public final class DashboardConstants {
        public final static boolean kBuddyClimbTelemetry = false;
        public final static boolean kClimberTelemetry = false;
        public final static boolean kDriveTelemetry = false;
        public final static boolean kHoodPIDTelemetry = true;
        public final static boolean kIndexerTelemetry = false;
        public final static boolean kIntakeArmTelemetry = false;
        public final static boolean kIntakeTelemetry = false;
        public final static boolean kLimelightTelemetry = false;
        public final static boolean kLoaderTelemetry = false;
        public final static boolean kPIDLoaderTelemetry = false;
        public final static boolean kServoTelemetry = false;
        public final static boolean kShooterPIDTelemetry = true;
    }
}
