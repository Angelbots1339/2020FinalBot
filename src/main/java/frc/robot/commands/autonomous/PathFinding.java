package frc.robot.commands.autonomous;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * PathFinding
 */
public class PathFinding {
        private PathFinding() {
                super();
        }

        public static Command getPath(DriveSubsystem drive) {
                // Trajectory command - not working yet
                // Need to create a basic command to drive forward

                // Create a voltage constraint to ensure we don't accelerate too fast
                // 10 V used here to account for battery sag
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts_WPI,
                                                DriveConstants.kvVoltSecondsPerMeter_WPI,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter_WPI),
                                DriveConstants.kDriveKinematics, DriveConstants.kMaxVolts);

                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSec_WPI,
                                DriveConstants.kMaxAccelerationMetersPerSec2_WPI)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics)
                                                // Apply the voltage constraint
                                                .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 0)),
                                // End 1 meter straight ahead of where we started, facing forward
                                new Pose2d(1, 0, new Rotation2d(0)),
                                // Pass config
                                config);
                return new RamseteCommand(exampleTrajectory, drive::getPose,
                                new RamseteController(DriveConstants.kRamseteB_WPI, DriveConstants.kRamseteZeta_WPI),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts_WPI,
                                                DriveConstants.kvVoltSecondsPerMeter_WPI,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter_WPI),
                                DriveConstants.kDriveKinematics, drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel_WPI, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel_WPI, 0, 0),
                                // RamseteCommand passes volts to the callback
                                drive::tankDriveVolts, drive).andThen(() -> drive.tankDriveVolts(0, 0));
        }
}