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
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIconstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.Auto;
import frc.robot.commands.ballmovement.LoaderToMiddleBB;
import frc.robot.commands.ballmovement.ReverseEverything;
import frc.robot.commands.ballmovement.ToggleIntakeArms;
import frc.robot.commands.vision.VisionShoot;
import frc.robot.subsystems.BuddyClimbSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.ShooterPID;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final IndexerSubsystem m_indexer = new IndexerSubsystem();
        private final IntakeSubsystem m_intake = new IntakeSubsystem();
        private final DriveSubsystem m_drive = new DriveSubsystem();
        private final ShooterPID m_rightShooterPID = new ShooterPID(ShooterConstants.kRightShooter, "Right Shooter",
                        true);
        private final ShooterPID m_leftShooterPID = new ShooterPID(ShooterConstants.kLeftShooter, "Left Shooter",
                        false);
        private final LoaderPIDSubsystem m_loaderPID = new LoaderPIDSubsystem(m_rightShooterPID);
        private final ClimberSubsystem m_climber = new ClimberSubsystem();
        private final BuddyClimbSubsystem m_servo = new BuddyClimbSubsystem(m_climber);
        private final HoodPIDSubsystem m_hood = new HoodPIDSubsystem();
        private final LimelightSubsystem m_limelight = new LimelightSubsystem();
        private final IntakeArmPID m_arm = new IntakeArmPID();

        XboxController m_driverController = new XboxController(OIconstants.kDriverControllerPort);
        XboxController m_testController = OIconstants.kTestControllerEnabled
                        ? new XboxController(OIconstants.kTestControllerPort)
                        : null;

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
                                new RunCommand(() -> m_drive.curvatureDrive(-m_driverController.getY(Hand.kLeft),
                                                m_driverController.getX(Hand.kRight)), m_drive));
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

                if(OIconstants.kTestControllerEnabled){
                        // Start Button --- Enable climbing
                        new JoystickButton(m_testController, Button.kStart.value).whenPressed(() -> m_climber.enable(),
                                        m_climber);
                        // B button --- climb
                        new JoystickButton(m_testController, Button.kB.value).whenPressed(() -> m_climber.runInverted(), m_climber)
                                        .whenReleased(() -> m_climber.stop(), m_climber);
                }

                /**
                 * DRIVER CONTROLLER -- Nick's prefered controls
                 */
                // right bumper --- Deter balls/run intake backwards
                new JoystickButton(m_driverController, Button.kBumperRight.value)
                                .whenPressed(() -> m_intake.reverseIntake())
                                .whenReleased(() -> m_intake.disableIntake());
                // left trigger --- Align to target
                // right trigger --- shoot all balls
                BooleanSupplier leftTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kLeft) > Constants.OIconstants.kLeftTriggerThreshold;
                BooleanSupplier rightTrigger = () -> m_driverController
                                .getTriggerAxis(Hand.kRight) > Constants.OIconstants.kRightTriggerThreshold;
                new Trigger(leftTrigger).or(new Trigger(rightTrigger))
                                .whileActiveOnce(new VisionShoot(m_intake, m_indexer, m_loaderPID, m_leftShooterPID,
                                                m_rightShooterPID, m_hood, m_limelight, m_drive, leftTrigger,
                                                rightTrigger, () -> m_driverController.getY(Hand.kLeft),
                                                LimelightConstants.kLongTimeout));
                // left bumper --- intake balls(balls to middle bb)
                new JoystickButton(m_driverController, Button.kBumperLeft.value)
                                .whenHeld(new LoaderToMiddleBB(m_loaderPID, m_intake, m_indexer));
                // A button -- Intake arm toggle
                new JoystickButton(m_driverController, Button.kA.value).whenPressed(new ToggleIntakeArms(m_arm));
                // Start Button --- Enable climbing
                new JoystickButton(m_driverController, Button.kStart.value).whenPressed(() -> m_climber.enable(),
                                m_climber);
                // B button --- climb
                new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_climber.run(), m_climber)
                                .whenReleased(() -> m_climber.stop(), m_climber);
                // X button --- unload/unjam
                new JoystickButton(m_driverController, Button.kX.value)
                                .whenHeld(new ReverseEverything(m_loaderPID, m_intake, m_indexer));
                // Y button --- deploy buddy climbing
                new JoystickButton(m_driverController, Button.kY.value).whenPressed(() -> m_servo.disengage(), m_servo);
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new Auto(m_arm, m_intake, m_indexer, m_loaderPID, m_leftShooterPID, m_rightShooterPID, m_hood,
                                m_limelight, m_drive);
        }
}
