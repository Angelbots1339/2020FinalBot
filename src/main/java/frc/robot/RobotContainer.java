/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIconstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.autonomous.Auto;
import frc.robot.commands.ballmovement.LoaderToMiddleBB2;
import frc.robot.commands.ballmovement.ReverseEverything;
import frc.robot.commands.ballmovement.ToggleIntakeArms;
import frc.robot.commands.vision.VisionShoot;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeArmPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LoaderPIDSubsystem;
import frc.robot.subsystems.ServoSubsystem;
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
    // private final LoaderSubsystem m_loader = new LoaderSubsystem();
    private final LoaderPIDSubsystem m_loaderPID = new LoaderPIDSubsystem();
    private final ShooterPID m_rightShooterPID = new ShooterPID(ShooterConstants.kRightShooter, "Right Shooter", true);
    private final ShooterPID m_leftShooterPID = new ShooterPID(ShooterConstants.kLeftShooter, "Left Shooter", false);
    private final ClimberSubsystem m_climber = new ClimberSubsystem();
    private final ServoSubsystem m_servo = new ServoSubsystem(m_climber);
    private final HoodPIDSubsystem m_hood = new HoodPIDSubsystem(8);
    private final LimelightSubsystem m_limelight = new LimelightSubsystem();
    private final IntakeArmPID m_arm = new IntakeArmPID();

    XboxController m_driverController = new XboxController(OIconstants.kDriverControllerPort);
    XboxController m_testController = new XboxController(OIconstants.kTestControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_drive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                // Left Y Axis needs to be inverted for driving forward
                new RunCommand(() -> m_drive.curvatureDrive(-1 * m_driverController.getY(Hand.kLeft),
                        m_driverController.getX(Hand.kRight)), m_drive));
        m_servo.engage();
        //m_arm.setSetpoint(0);
        //m_arm.enable();

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

        // new JoystickButton(m_testController, Button.kA.value).whenHeld(new
        // RunHood(m_hood, -.1));
        // new JoystickButton(m_testController, Button.kB.value).whenHeld(new
        // RunHood(m_hood, 14));

        // new JoystickButton(m_testController, Button.kBumperRight.value)
        // .whenHeld(new LoaderToMiddleBB2(m_loaderPID, m_intake, m_indexer));
        // new JoystickButton(m_testController, Button.kBumperLeft.value)
        // .whenHeld(new RunShooter(m_leftShooterPID, m_rightShooterPID, 5000));

        /**
         * DRIVER CONTROLLER
         */

        // drives servos with Button
        // Left Bumper - Intakes - REVERSE INTAKE
        // new JoystickButton(m_driverController, Button.kBumperLeft.value)
        // .whenHeld(new LoaderToMiddleBB(m_loader, m_intake, m_indexer));
        // // Right Bumper - Shoots - MOVE INTAKE ARM
        // new JoystickButton(m_driverController,
        // Button.kBumperRight.value).whenHeld(new RunVision(m_limelight, m_drive,
        // m_leftShooterPID, m_rightShooterPID, m_intake, m_indexer, m_loader, m_hood));
        // // A Button -
        // new JoystickButton(m_driverController, Button.kA.value)
        // .whenHeld(new MoveBallsToShooter(m_intake, m_indexer, m_loader));
        // // B Button - Reeve up Shooter
        // new JoystickButton(m_driverController, Button.kB.value)
        // .whenHeld(new RunShooter(m_rightShooterPID, m_leftShooterPID, 4000));
        // // X Button - Reverse indexer
        // new JoystickButton(m_driverController, Button.kX.value).whenPressed(() ->
        // m_indexer.reverse())
        // .whenReleased(() -> m_indexer.disable());
        // // Y Button - reverses intake - REVERSE EVERYTHING
        // new JoystickButton(m_driverController, Button.kY.value).whenPressed(() ->
        // m_intake.reverseIntake())
        // .whenReleased(() -> m_intake.disableIntake());

        // Right trigger - SHOOT ALL BALLS
        // Left trigger - INTAKE

        /**
         * Nick's prefered controls
         */
        // right bumper --- Deter balls/run intake backwards
        new JoystickButton(m_driverController, Button.kBumperRight.value).whenPressed(() -> m_intake.reverseIntake())
                .whenReleased(() -> m_intake.disableIntake());
        // right trigger --- shoot all balls
        new Trigger(() -> m_driverController.getTriggerAxis(Hand.kLeft) > Constants.OIconstants.kLeftTriggerThreshold
                || m_driverController.getTriggerAxis(Hand.kRight) > Constants.OIconstants.kRightTriggerThreshold)
                        .whileActiveOnce(new VisionShoot(m_intake, m_indexer, m_loaderPID, m_leftShooterPID,
                                m_rightShooterPID, m_hood, m_limelight, m_drive,
                                () -> m_driverController
                                        .getTriggerAxis(Hand.kLeft) > Constants.OIconstants.kLeftTriggerThreshold,
                                () -> m_driverController
                                        .getTriggerAxis(Hand.kRight) > Constants.OIconstants.kRightTriggerThreshold,
                                () -> m_driverController.getY(Hand.kLeft), 60));
        // left bumper --- intake balls(balls to middle bb)
        new JoystickButton(m_driverController, Button.kBumperLeft.value)
                .whenHeld(new LoaderToMiddleBB2(m_loaderPID, m_intake, m_indexer));
        // A button -- Intake arm toggle
        new JoystickButton(m_driverController, Button.kA.value).whenPressed(new ToggleIntakeArms(m_arm));
        // B button --- climb
        new JoystickButton(m_driverController, Button.kStart.value).whenPressed(() -> m_climber.enable(), m_climber);

        new JoystickButton(m_driverController, Button.kB.value).whenPressed(() -> m_climber.run(), m_climber)
                .whenReleased(() -> m_climber.stop(), m_climber);
        // X button --- unload/unjam
        new JoystickButton(m_driverController, Button.kX.value)
                .whenHeld(new ReverseEverything(m_loaderPID, m_intake, m_indexer));

        // Y button --- deploy buddy climbing
        new JoystickButton(m_driverController, Button.kY.value).whenPressed(() -> m_servo.disengage(), m_servo);
        // .whenReleased(() -> m_climber.disable()); ---doesn't work yet

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
