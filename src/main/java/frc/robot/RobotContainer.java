/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIconstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.Auto;
import frc.robot.commands.autonomous.AutoTest;
import frc.robot.commands.autonomous.ControlledAuto;
import frc.robot.commands.autonomous.PIDDrive;
import frc.robot.commands.ballmovement.LoaderToMiddleBB;
import frc.robot.commands.ballmovement.ReverseEverything;
import frc.robot.commands.ballmovement.ToggleIntakeArms;
import frc.robot.commands.utils.DriveControl;
import frc.robot.commands.vision.VisionShoot;
import frc.robot.subsystems.BuddyClimbSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakingSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSide;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final DriveSubsystem m_drive = new DriveSubsystem();
        private final Shooter m_shooter = new Shooter(
                        new ShooterSide(ShooterConstants.kRightShooter, "Right Shooter", true),
                        new ShooterSide(ShooterConstants.kLeftShooter, "Left Shooter", false));
        private final IntakingSystem m_intaker = new IntakingSystem(new IndexerSubsystem(), new IntakeSubsystem(),
                        new LoaderPIDSubsystem(), m_shooter);
        private final ClimberSubsystem m_climber = new ClimberSubsystem();
        private final BuddyClimbSubsystem m_servo = new BuddyClimbSubsystem(m_climber);
        private final HoodPIDSubsystem m_hood = new HoodPIDSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem();
        private final IntakeArmPID m_arm = new IntakeArmPID();

        XboxController m_driverController = new XboxController(OIconstants.kDriverControllerPort);
        XboxController m_testController = OIconstants.kTestControllerEnabled
                        ? new XboxController(OIconstants.kTestControllerPort)
                        : null;

        private final DriveControl m_driveControl = new DriveControl(() -> -m_driverController.getY(Hand.kLeft),
                        () -> m_driverController.getX(Hand.kRight));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                for (String key : SmartDashboard.getKeys())
                        SmartDashboard.delete(key);

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                m_drive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                // Left Y Axis needs to be inverted for driving forward
                                new RunCommand(() -> m_drive.curvatureDrive(m_driveControl), m_drive));
                new RunCommand(() -> m_intaker.telemetry()).schedule();// TODO
                m_servo.engage();
                m_arm.setEncoderPosition(-1);
                // m_arm.setSetpoint(0);
                // m_arm.enable();

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by instantiating a {@link GenericHID} or one of its subclasses
         * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
         * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                /**
                 * TEST CONTROLLER
                 * 
                 */
                if (!OIconstants.kTestControllerEnabled && Math.floor(1) == 0) {
                        int i = 0; // generate warning if test controller is enabled
                        i = i + 0;
                }

                if (OIconstants.kTestControllerEnabled) {
                        // Start Button --- Enable climbing
                        new JoystickButton(m_testController, Button.kStart.value).whenPressed(() -> m_climber.enable(),
                                        m_climber);
                        // B button --- climb
                        new JoystickButton(m_testController, Button.kB.value)
                                        .whenPressed(() -> m_climber.runInverted(), m_climber)
                                        .whenReleased(() -> m_climber.stop(), m_climber);

                }
                new JoystickButton(m_testController, Button.kA.value)
                                .whenPressed(new PIDDrive(m_drive, 90, 0, () -> (m_drive.getRotation()),
                                                () -> (m_drive.getLeftMeters() + m_drive.getRightMeters()), 1));
                new JoystickButton(m_testController, Button.kX.value)
                                .whenPressed(new PIDDrive(m_drive, -90, 0, () -> (m_drive.getRotation()),
                                                () -> (m_drive.getLeftMeters() + m_drive.getRightMeters()), 1));
                new JoystickButton(m_testController, Button.kY.value)
                                .whenPressed(new PIDDrive(m_drive, 0, 1, () -> (m_drive.getRotation()),
                                                () -> (m_drive.getLeftMeters() + m_drive.getRightMeters()), 1));

                /**
                 * DRIVER CONTROLLER -- Nick's prefered controls
                 */
                // right bumper --- Deter balls/run intake backwards
                new JoystickButton(m_driverController, Button.kBumperRight.value)
                                .whenPressed(() -> m_intaker.reverseIntake())
                                .whenReleased(() -> m_intaker.disableIntake());
                // left trigger --- Align to target
                // right trigger --- shoot all balls
                BooleanSupplier leftTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kLeft) > Constants.OIconstants.kLeftTriggerThreshold;
                BooleanSupplier rightTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kRight) > Constants.OIconstants.kRightTriggerThreshold;
                new Trigger(leftTrigger).or(new Trigger(rightTrigger))
                                .whileActiveOnce(new VisionShoot(m_intaker, m_shooter, m_hood, m_limelight, m_drive,
                                                leftTrigger, rightTrigger, m_driveControl,
                                                LimelightConstants.kLongTimeout));
                // back button --- shoot line no vision
                new JoystickButton(m_driverController, Button.kBack.value).whileActiveOnce(new VisionShoot(m_intaker,
                                m_shooter, m_hood, m_limelight, m_drive, () -> false, () -> true, DriveControl.empty,
                                LimelightConstants.kLongTimeout, false, 3));
                // right stck down --- shoot close no vision
                new JoystickButton(m_driverController, Button.kStickRight.value).whileActiveOnce(new VisionShoot(
                                m_intaker, m_shooter, m_hood, m_limelight, m_drive, () -> false, () -> true,
                                DriveControl.empty, LimelightConstants.kLongTimeout, false, 0));
                // left bumper --- intake balls(balls to middle bb)
                new JoystickButton(m_driverController, Button.kBumperLeft.value)
                                .whenHeld(new LoaderToMiddleBB(m_intaker));
                // A button -- Intake arm toggle
                new JoystickButton(m_driverController, Button.kA.value).whenPressed(new ToggleIntakeArms(m_arm));
                // Start Button --- Enable climbing
                new JoystickButton(m_driverController, Button.kStart.value).whenPressed(() -> m_climber.enable(),
                                m_climber);
                // B button --- climb
                new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_climber.run(), m_climber)
                                .whenReleased(() -> m_climber.stop(), m_climber);
                // X button --- unload/unjam
                new JoystickButton(m_driverController, Button.kX.value).whenHeld(new ReverseEverything(m_intaker));
                // Y button --- deploy buddy climbing
                new JoystickButton(m_driverController, Button.kY.value).whenPressed(() -> m_servo.disengage(), m_servo);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                //return autos.timeout;

                

                // Trajectory command - not working yet
                // Need to create a basic command to drive forward

                // Create a voltage constraint to ensure we don't accelerate too fast
                // 10 V used here to account for battery sag
                var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                                new SimpleMotorFeedforward(DriveConstants.ksVolts_WPI, DriveConstants.kvVoltSecondsPerMeter_WPI,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter_WPI),
                                DriveConstants.kDriveKinematics, 10);

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
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(1, 0)
                                ),
                        // End 2 meters straight ahead of where we started, facing forward
                        new Pose2d(2, 0, new Rotation2d(0)),
                        // Pass config
                        config);
                RamseteCommand ramseteCommand = new RamseteCommand(
                        exampleTrajectory, 
                        m_drive::getPose,
                        new RamseteController(DriveConstants.kRamseteB_WPI, DriveConstants.kRamseteZeta_WPI),
                        new SimpleMotorFeedforward(DriveConstants.ksVolts_WPI, 
                                DriveConstants.kvVoltSecondsPerMeter_WPI,
                                DriveConstants.kaVoltSecondsSquaredPerMeter_WPI),
                        DriveConstants.kDriveKinematics, 
                        m_drive::getWheelSpeeds,
                        new PIDController(DriveConstants.kPDriveVel_WPI, 0, 0),
                        new PIDController(DriveConstants.kPDriveVel_WPI, 0, 0),
                        // RamseteCommand passes volts to the callback
                        m_drive::tankDriveVolts, 
                        m_drive);

                // Run path following command, then stop at the end.
                return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));


        }

        Autos autos = new Autos();

        @SuppressWarnings("unused")
        private class Autos {
                // public final Command timeout = new Auto(m_arm, m_intaker, m_shooter, m_hood,
                // m_limelight, m_drive),
                // encoder = new ControlledAuto(m_arm, m_intaker, m_shooter, m_hood,
                // m_limelight, m_drive);
                public final Command timeout = new AutoTest(m_drive);
        }

        public void resetEncoders() {
                m_drive.resetEncoders();
                m_drive.zeroHeading();
        }
}
