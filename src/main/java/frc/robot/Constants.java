/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    private Constants() {
    }

    public static final class DriveConstants {
        private DriveConstants() {
        }

        // TalonFX Motors
        public static final int kLeftFrontMotor = 13;
        public static final int kLeftBackMotor = 14;
        public static final int kRightFrontMotor = 12;
        public static final int kRightBackMotor = 15;

        public static final int kClicksPerRotation = 2048;
        public static final double kGearRatio = 5.45; // based on drivetrain design
        public static final double kWheelDiameterMeter = 0.0990; // wheels are just under 4"
        public static final double kMetersPerClick = Math.PI * kWheelDiameterMeter / kClicksPerRotation / kGearRatio;

        public static final double kMaxDriveSpeed = 0.6; // speed

        public static final double kMinPower = 0.2; // speed
        public static final double kTurnInPlaceThreshold = 0.2;

        public static final double kPDrive = 0.0008;
        public static final double kIDrive = 0.0;
        public static final double kDDrive = 0.0;
        public static final double kPTurn = 0.0008;
        public static final double kITurn = 0.0;
        public static final double kDTurn = 0.0;

        // Values here are from Characterization Tool
        public static final double ksVolts_WPI = 0.288;
        public static final double kvVoltSecondsPerMeter_WPI = 1.89;
        public static final double kaVoltSecondsSquaredPerMeter_WPI = 0.411;
        public static final double kPDriveVel_WPI = 8; // was 15 this seems high....
        public static final double kMaxSpeedMetersPerSec_WPI = 1.5; // limit for now
        public static final double kMaxAccelerationMetersPerSec2_WPI = 0.5; // limit for now
        public static final double kRamseteB_WPI = 2;
        public static final double kRamseteZeta_WPI = 0.7;
        public static final boolean kGyroReversed = false;
        public static final double kTrackWidthMeters = 0.82;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackWidthMeters);
        public static final double kMaxVolts = 10;

    }

    public static final class IntakeConstants {
        private IntakeConstants() {
        }

        public static final int kIntakeMotor = 5;
        public static final int kLeftIntakeMoverMotor = 9; // not inverted in fireware
        public static final int kRightIntakeMoverMotor = 8; // inverted in firmware

        public static final double kMaxIntakeSpeed = 1;
        public static final double kMaxIntakeArmSpeed = 0.15;

        public static final double kMaxNormalVoltage = 3;
        public static final double kMaxVeloValue = 0.3;
        public static final double kMaxNormalCurrent = 12;

        // Intake Arm Constants
        public static final double kP = 0.4;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kIntakeArmMotorVolt = 5;
        public static final double kMinEncoderValue = -10.5;
        public static final double kMaxEncoderValue = -0.5;
        public static final double kPositionTolerance = 0.2;
        public static final double kMidEncoderValue = (kMinEncoderValue + kMaxEncoderValue) / 2;
    }

    public static final class IndexerConstants {
        private IndexerConstants() {
        }

        public static final int kLeftIndexerMotor = 7;
        public static final int kRightIndexMotor = 6;
        public static final double kMaxIndexSpeed = 0.7;
        public static final double kAcceptableCurrentSpike = 0;

    }

    public static final class LoaderConstants {
        private LoaderConstants() {
        }

        public static final int kLoaderMotor = 2;
        // This speed is used when feeding balls to shooter
        public static final double kFeedLoaderSpeed = 0.6;
        // This speed is used when loading first ball
        public static final double kInitLoaderSpeed = 0.45;
        // This speed is used when reversing
        public static final double kReverseLoaderSpeed = 0.20;
        public static final double kFullSpeed = 1;
        // Loader PID
        public static final double kP = 0.007;
        public static final double kI = 0.000;
        public static final double kD = 0.001;
        public static final double kLoaderToleranceRPM = 10;
        public static final double kLoaderSetpoint = 920;
        public static final double KSVolts = 0.25;
        public static final double KVVoltSecondsPerRotation = 12 / (11000 / 4.8);
        public static final double kVelocityConversionFactor = 0.2083;
        public static final int kMaxBalls = 5;

    }

    public static final class ShooterConstants {
        private ShooterConstants() {
        }

        public static final int kLeftShooter = 1;
        public static final int kRightShooter = 3;

        public static final double kRapidShotThreshold = 9; // in meters
        // PID values
        public static final double kP = 0.0008; // all good
        public static final double kI = 0.00001;
        public static final double kD = 0;
        public static final double kFF = 0;

        // setting up encoder
        public static final int kEncoderCPR = 4096; // clicks per rotation

        // distance units int rotations
        public static final double kEncoderDistancePerPulse = 1.0 / (double) kEncoderCPR;
        public static final double kShooterFreeRPM = 5676;
        public static final double kShooterTargetRPM = 5450; // ~5500 is max speed, rotations per second, 3 to 2 gear
                                                             // ratio
        public static final double kBaseRPM = 2500;
        public static final double kShooterToleranceSlope = 22. / 1000;

        // 0.18 to 0.23 seems to be a sweet spot, varies based on Battery?
        // ~0.23 was found empirically, working well prior to Utah.
        // Post Utah, not hitting set point, changed from 0.23 to 0.37
        public static final double KSVolts = 0.23;
        public static final double KVVoltSecondsPerRotation = 12 / kShooterFreeRPM; // 12 V is working well
        public static final double kMaxShooterSpeed = 0.5;

    }

    public static final class HoodConstants {
        private HoodConstants() {
        }

        public static final int kHoodPort = 4;

        public static final double kMaxVeloValue = 0.2;
        public static final double kMaxNormalCurrent = 18;
        // PID HOOD
        public static final double kP = 0.9; // 0.8
        public static final double kI = 0.0;
        public static final double kD = 0.019;
        public static final double kMaxHoodVolt = 1;
        public static final double kMinEncoderValue = 0;
        public static final double kmaxEncoderValue = 17.4;

        public static final double kMinResistedVoltage = 60;
        public static final double kPositionTolerance = .3;
        public static final double KSVolts = 0.15;
        public static final double KVVoltSecondsPerRotation = 12 * (12.5 / 11000); // Neo 550 - 11000 rpm, with 12.5:1
                                                                                   // reduction

        public static final double kLowSetpoint = 0;
    }

    public static final class ClimberConstants {
        private ClimberConstants() {
        }

        public static final int kLeftClimberMotor = 10;
        public static final int kRightClimberMotor = 11;

        public static final double kClimberSpeed = 0.7;

        public static final double kSVolts = 0;
        public static final double kSVoltsPerSecondsPerRotation = 0;

        public static final double kClimberToleranceRPS = 0;
        public static final double kEncoderDistancePerPulse = 0;
        public static final double kClimberTargetRPS = 0;
        public static final double kClimberResetSpeed = 0.35;
    }

    public static final class BuddyClimbConstants {
        private BuddyClimbConstants() {
        }

        public static final int kLeftServo = 0;
        public static final int kRightServo = 6;
        public static final double kLeftEngagePos = 10;
        public static final double kLeftDisengagePos = 110;
        public static final double kRightEngagePos = 154;
        public static final double kRightDisengagePos = 54;
    }

    public static final class LimelightConstants {
        private LimelightConstants() {
        }

        // Ensure that the limelight hostname is just "limelight", otherwise there will
        // be nothing on networktable for data. Check network table data in Shuffleboard
        // when troubleshooting. CHECK THE TEAM NUMBER
        public static final String kLimeTable = "limelight";
        public static final int kDefaultPipeline = 0;
        public static final double kDriveP = 0.25;
        public static final double kDriveTolerance = 0.3;// used to be 0.055
        public static final double kShootTolerance = 0.4;// used to be 0.055
        public static final double kLimelightHeight = 0.43;
        public static final double kTagetHeight = 2.08;
        public static final double kLimelightToTargetHeight = kTagetHeight - kLimelightHeight;
        public static final double kVerticalFOV = 49.7;
        public static final double kHorizontalFOV = 59.6;
        public static final double kLimeLightTilt = 25;
        public static final double kDefaultDistance = 3;

        public static final double k2XZoomCutoff = 3.785;
        public static final double k3XZoomCutoff = 5.436;
        public static final double[] kPanningOffest = { 0, -9, -6, 0 };
        public static final boolean kAutoZoom = true;

        public static final boolean kAutoLight = false;

        public static final boolean kAutoColorVision = false;
        public static final boolean kDistanceAlign = false;
        public static final int kColorPipeline = 3;
    }

    public static final class SensorConstants {
        private SensorConstants() {
        }

        public static final int kTopEmitter = 5;
        public static final int kTopReciever = 4;
        public static final int kMiddleEmitter = 3;
        public static final int kMiddleReciever = 2;
        public static final int kShooterEmitter = 0;// right next to shooter, counts when balls leave
        public static final int kShooterReciever = 1;
        public static final int kBottomEmitter = 7; // counts when balls enter
        public static final int kBottomReciever = 6;

    }

    public static final class AutoConstants {
        private AutoConstants() {
        }

        public static final double kReverseTime = 1.5;
        public static final double kVisionTime = 4;
        public static final double kReverseSpeed = 0.62;
        public static final double kPIDReverseTime = 3;
    }

    public static final class OIconstants {
        private OIconstants() {
        }

        // Main Joystick USB Port
        public static final int kDriverControllerPort = 0;
        public static final int kTestControllerPort = 2;

        public static final double kLeftTriggerThreshold = 0.5;
        public static final double kRightTriggerThreshold = 0.5;

        public static final boolean kTestControllerEnabled = false;
    }

    public static final class DashboardConstants {
        private DashboardConstants() {
        }

        public static final boolean kBuddyClimbTelemetry = false;
        public static final boolean kClimberTelemetry = false;
        public static final boolean kDriveTelemetry = true;
        public static final boolean kHoodPIDTelemetry = true;
        public static final boolean kIndexerTelemetry = false;
        public static final boolean kIntakeArmTelemetry = false;
        public static final boolean kIntakeTelemetry = false;
        public static final boolean kLimelightTelemetry = false;
        public static final boolean kLoaderTelemetry = false;
        public static final boolean kShooterPIDTelemetry = true;
        public static final boolean kExcessDriveTelemetry = true;
    }

    public static final class FieldConstants {
        private FieldConstants() {
        }

        public static final double kStarterLine = 3;
    }

    public static final class AngleConstants {
        private AngleConstants() {
        }

        public static final double kFullTurn = 360;
        public static final double kHalfTurn = kFullTurn / 2;
        public static final double kQuarterTurn = kFullTurn / 4;
    }

    public static final class MotorConstants {
        private MotorConstants() {
        }

        public static final double kFalconUpdateTime = 0.1;
    }
}
