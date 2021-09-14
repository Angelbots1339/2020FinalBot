/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIconstants;
import frc.robot.commands.autonomous.Auto;
import frc.robot.commands.autonomous.AutoTest;
import frc.robot.commands.autonomous.ControlledAuto;
import frc.robot.commands.autonomous.PIDDrive;
import frc.robot.commands.autonomous.PathFinding;
import frc.robot.commands.ballmovement.LoaderToMiddleBB;
import frc.robot.commands.ballmovement.ToggleIntakeArms;
import frc.robot.commands.utils.DriveControl;
import frc.robot.commands.utils.VisionControl;
import frc.robot.commands.vision.VisionShoot;
import frc.robot.subsystems.BuddyClimbSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakingSystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Shooter;

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
        private final Shooter m_shooter = new Shooter();
        private final IntakingSystem m_intaker = new IntakingSystem(m_shooter);
        private final ClimberSubsystem m_climber = new ClimberSubsystem();
        private final BuddyClimbSubsystem m_servo = new BuddyClimbSubsystem(m_climber);
        private final HoodPIDSubsystem m_hood = new HoodPIDSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem();
        private final IntakeArmPID m_arm = new IntakeArmPID();

        XboxController m_driverController = new XboxController(OIconstants.kDriverControllerPort);
        XboxController m_testController = OIconstants.kTestControllerEnabled
                        ? new XboxController(OIconstants.kTestControllerPort)
                        : new XboxController(OIconstants.kDriverControllerPort);

        private final DriveControl m_driveControl = new DriveControl(() -> -m_driverController.getY(Hand.kLeft),
                        () -> m_driverController.getX(Hand.kRight));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                SmartDashboard.getKeys().forEach(SmartDashboard::delete);

                // Configure default commands
                // Set the default drive command to split-stick arcade drive
                m_drive.setDefaultCommand(
                                // A split-stick arcade command, with forward/backward controlled by the left
                                // hand, and turning controlled by the right.
                                // Left Y Axis needs to be inverted for driving forward
                                new RunCommand(() -> m_drive.curvatureDrive(m_driveControl), m_drive));
                new RunCommand(m_intaker::telemetry).schedule();// TODO
                m_servo.engage();
                m_arm.setEncoderPosition(-1);

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

                if (OIconstants.kTestControllerEnabled) {
                        // Start Button --- Enable climbing
                        new JoystickButton(m_testController, Button.kStart.value).whenPressed(m_climber::enable,
                                        m_climber);
                        // B button --- climb
                        new JoystickButton(m_testController, Button.kB.value)
                                        .whenPressed(m_climber::runInverted, m_climber)
                                        .whenReleased(m_climber::stop, m_climber);

                }
                
                new JoystickButton(m_testController, Button.kA.value).whileHeld(new PIDDrive(m_drive, AngleConstants.kQuarterTurn, 0,
                                new DriveControl(m_drive::getRotation, m_drive::getForwardMeters)));
                new JoystickButton(m_testController, Button.kX.value).whileHeld(new PIDDrive(m_drive, -AngleConstants.kQuarterTurn, 0,
                                new DriveControl(m_drive::getRotation, m_drive::getForwardMeters)));
                new JoystickButton(m_testController, Button.kY.value).whileHeld(new PIDDrive(m_drive, 0, 1,
                                new DriveControl(m_drive::getRotation, m_drive::getForwardMeters)));

                /**
                 * DRIVER CONTROLLER -- Nick's prefered controls
                 */
                // right bumper --- Deter balls/run intake backwards
                new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(m_intaker::reverseIntake)
                                .whenReleased(m_intaker::disableIntake);
                // left trigger --- Align to target
                // right trigger --- shoot balls
                BooleanSupplier leftTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kLeft) > Constants.OIconstants.kLeftTriggerThreshold;
                BooleanSupplier rightTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kRight) > Constants.OIconstants.kRightTriggerThreshold;
                new Trigger(leftTrigger).or(new Trigger(rightTrigger))
                                .whileActiveOnce(new VisionShoot(m_intaker, m_shooter, m_hood, m_limelight, m_drive,
                                                new VisionControl(leftTrigger, rightTrigger), m_driveControl));
                // back button --- shoot line no vision
                new JoystickButton(m_driverController, Button.kBack.value).whileActiveOnce(new VisionShoot(m_intaker,
                                m_shooter, m_hood, m_limelight, m_drive, DriveControl.empty, FieldConstants.kStarterLine));
                // right stick down --- shoot close no vision
                new JoystickButton(m_driverController, Button.kStickRight.value).whileActiveOnce(new VisionShoot(
                                m_intaker, m_shooter, m_hood, m_limelight, m_drive, DriveControl.empty, 0));
                // left bumper --- intake balls(balls to middle bb)
                new JoystickButton(m_driverController, Button.kBumperLeft.value)
                                .whenHeld(new LoaderToMiddleBB(m_intaker));
                // A button -- Intake arm toggle
                new JoystickButton(m_driverController, Button.kA.value).whenPressed(new ToggleIntakeArms(m_arm));
                // Start Button --- Enable climbing
                new JoystickButton(m_driverController, Button.kStart.value).whenPressed(m_climber::enable, m_climber);
                // B button --- climb
                new JoystickButton(m_driverController, Button.kB.value).whenPressed(m_climber::run, m_climber)
                                .whenReleased(m_climber::stop, m_climber);
                // X button --- unload/unjam
                new JoystickButton(m_driverController, Button.kX.value)
                                .whenPressed(m_intaker::reverse, m_intaker.getRequirements())
                                .whenReleased(m_intaker::disable, m_intaker.getRequirements());
                // Y button --- deploy buddy climbing
                new JoystickButton(m_driverController, Button.kY.value).whenPressed(m_servo::disengage, m_servo);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autos.pathfinding;

        }

        Autos autos = new Autos();

        @SuppressWarnings("unused")
        private class Autos {
                public final Command timeout = new Auto(m_arm, m_intaker, m_shooter, m_hood, m_limelight, m_drive);
                public final Command encoder = new ControlledAuto(m_arm, m_intaker, m_shooter, m_hood, m_limelight,
                                m_drive);
                public final Command timeout2 = new AutoTest(m_drive);
                public final Command pathfinding = PathFinding.getPath(m_drive);
        }

        public void resetEncoders() {
                m_drive.resetEncoders();
                m_drive.zeroHeading();
        }
}
