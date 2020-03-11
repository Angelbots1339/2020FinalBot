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

        public final static int kClicksPerRotation = 2048;
        public final static double kGearRatio = 5.45;
        public final static double kWheelDiameter = 0.1016;
        public final static double kMetersPerClick = Math.PI * kWheelDiameter / kClicksPerRotation / kGearRatio;

        public final static double kMaxDriveSpeed = 0.6; // speed

        public final static double kMinPower = 0.2; // speed
        public final static double kTurnInPlaceThreshold = 0.2;

        public final static double kPDrive = 0.1;
        public final static double kIDrive = 0.0;
        public final static double kDDrive = 0.0;
        public final static double kPTurn = 0.1;
        public final static double kITurn = 0.0;
        public final static double kDTurn = 0.0;
    }

    public final class IntakeConstants {
        public final static int kIntakeMotor = 5;
        public final static int kLeftIntakeMoverMotor = 9; // not inverted in fireware
        public final static int kRightIntakeMoverMotor = 8; // inverted in firmware

        public final static double kMaxIntakeSpeed = 1;
        public final static double kMaxIntakeArmSpeed = 0.15;

        public final static double kMaxNormalVoltage = 3;
        public final static double kMaxVeloValue = 0.3;
        public final static double kMaxNormalCurrent = 12;

        // Intake Arm Constants
        public final static double kP = 0.4;
        public final static double kI = 0;
        public final static double kD = 0;
        public final static double kIntakeArmMotorVolt = 5;
        public final static double kMinEncoderValue = -10.5;
        public final static double kMaxEncoderValue = -0.5;
        public final static double kPositionTolerance = 0.2;
    }

    public final class IndexerConstants {
        public final static int kLeftIndexerMotor = 7;
        public final static int kRightIndexMotor = 6;
        public final static double kMaxIndexSpeed = 0.7;
        public final static double kAcceptableCurrentSpike = 0;

    }

    public final class LoaderConstants {
        public final static int kLoaderMotor = 2;
        // This speed is used when feeding balls to shooter
        public final static double kFeedLoaderSpeed = 0.6;
        // This speed is used when loading first ball
        public final static double kInitLoaderSpeed = 0.45;
        // This speed is used when reversing
        public final static double kReverseLoaderSpeed = 0.20;
        public final static double kFullSpeed = 1;
        // Loader PID
        public final static double kP = 0.007;
        public final static double kI = 0.000;
        public final static double kD = 0.001;
        public final static double kLoaderToleranceRPM = 10;
        public final static double kLoaderSetpoint = 920;
        public final static double KSVolts = 0.25;
        public final static double KVVoltSecondsPerRotation = 12 / (11000 / 4.8);

    }

    public final static class ShooterConstants {
        public final static int kLeftShooter = 1;
        public final static int kRightShooter = 3;

        public final static double kRapidShotThreshold = 9; // in meters
        // PID values
        public final static double kP = 0.0008; // all good
        public final static double kI = 0.00001;
        public final static double kD = 0;
        public final static double kFF = 0;

        // setting up encoder
        public final static int kEncoderCPR = 4096; // clicks per rotation

        // distance units int rotations
        public final static double kEncoderDistancePerPulse = 1.0 / (double) kEncoderCPR;
        public final static double kShooterFreeRPM = 5676;
        public final static double kShooterTargetRPM = 5450; // ~5500 is max speed, rotations per second, 3 to 2 gear
                                                             // ratio
        public static final double kBaseRPM = 2500;
        public final static double kShooterToleranceSlope = 22. / 1000;

        // 0.18 to 0.23 seems to be a sweet spot, varies based on Battery?
        // ~0.23 was found empirically, working well prior to Utah.
        // Post Utah, not hitting set point, changed from 0.23 to 0.37
        public final static double KSVolts = 0.23; 
        public final static double KVVoltSecondsPerRotation = 12 / kShooterFreeRPM; // 12 V is working well
        public final static double kMaxShooterSpeed = 0.5;

    }

    public final class HoodConstants {
        public final static int kHoodPort = 4;

        public final static double kMaxVeloValue = 0.2;
        public final static double kMaxNormalCurrent = 18;
        // PID HOOD
        public final static double kP = 0.9; // 0.8
        public final static double kI = 0.0;
        public final static double kD = 0.019;
        public final static double kMaxHoodVolt = 1;
        public final static double kMinEncoderValue = 0;
        public final static double kmaxEncoderValue = 17.4;

        public final static double kMinResistedVoltage = 60;
        public final static double kPositionTolerance = .3;
        public final static double KSVolts = 0.15;
        public final static double KVVoltSecondsPerRotation = 12 * (12.5 / 11000); // Neo 550 - 11000 rpm, with 12.5:1
                                                                                   // reduction

        public static final double kLowSetpoint = 0;
    }

    public final static class ClimberConstants {
        public final static int kLeftClimberMotor = 10;
        public final static int kRightClimberMotor = 11;

        public final static double kClimberSpeed = 0.7;

        public final static double kSVolts = 0;
        public final static double kSVoltsPerSecondsPerRotation = 0;

        public final static double kClimberToleranceRPS = 0;
        public final static double kEncoderDistancePerPulse = 0;
        public final static double kClimberTargetRPS = 0;
        public static final double kClimberResetSpeed = 0.35;
    }

    public final class BuddyClimbConstants {
        public final static int kLeftServo = 0;
        public final static int kRightServo = 6;
        public final static double kLeftEngagePos = 10;
        public final static double kLeftDisengagePos = 110;
        public final static double kRightEngagePos = 154;
        public final static double kRightDisengagePos = 54;
    }

    public final static class LimelightConstants {
        // Ensure that the limelight hostname is just "limelight", otherwise there will
        // be nothing on networktable for data. Check network table data in Shuffleboard
        // when troubleshooting. CHECK THE TEAM NUMBER
        public final static String kLimeTable = "limelight";
        public final static int kDefaultPipeline = 0;
        public final static double kDriveP = 0.25;
        public final static double kDriveTolerance = 0.3;// used to be 0.055
        public final static double kShootTolerance = 0.4;// used to be 0.055
        public final static double kLimelightHeight = 0.43;
        public final static double kTagetHeight = 2.08;
        public final static double kLimelightToTargetHeight = kTagetHeight - kLimelightHeight;
        public final static double kVerticalFOV = 49.7;
        public final static double kHorizontalFOV = 59.6;
        public final static double kLimeLightTilt = 25;
        public final static double kDefaultDistance = 3;

        public final static double k2XZoomCutoff = 3.785;
        public final static double k3XZoomCutoff = 5.436;
        public final static double[] kPanningOffest = { 0, -9, -6, 0 };
        public final static boolean kAutoZoom = true;

        public final static boolean kAutoLight = false;

        public static final boolean kAutoColorVision = false;
        public static final boolean kDistanceAlign = false;
        public static final double kLongTimeout = 60;
        public static final int kColorPipeline = 3;
    }

    public final class SensorConstants {

        public final static int kTopEmitter = 5;
        public final static int kTopReciever = 4;
        public final static int kMiddleEmitter = 3;
        public final static int kMiddleReciever = 2;
        public final static int kShooterEmitter = 0;// right next to shooter, counts when balls leave
        public final static int kShooterReciever = 1;
        public final static int kBottomEmitter = 7; // counts when balls enter
        public final static int kBottomReciever = 6;

    }

    public final static class AutoConstants {
        public final static double kReverseTime = 1.5;
        public static final double kVisionTime = 4;
        public static final double kReverseSpeed = 0.62;
    }

    public final class OIconstants {

        // Main Joystick USB Port
        public final static int kDriverControllerPort = 0;
        public final static int kTestControllerPort = 2;

        public final static double kLeftTriggerThreshold = 0.5;
        public final static double kRightTriggerThreshold = 0.5;

        public final static boolean kTestControllerEnabled = true;
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
        public final static boolean kShooterPIDTelemetry = true;
        public static final boolean kExcessDriveTelemetry = false;
    }
}
