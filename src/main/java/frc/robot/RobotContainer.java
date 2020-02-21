/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIconstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.HoodPID;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.AdjustableHoodSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ServoTest;
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
  private final LoaderSubsystem m_loader = new LoaderSubsystem();
  private final ShooterPID m_rightShooterPID = new ShooterPID(ShooterConstants.kRightShooter, "Right Shooter", true);
  private final ShooterPID m_leftShooterPID = new ShooterPID(ShooterConstants.kLeftShooter, "Left Shooter", false);
  private AdjustableHoodSubsystem m_hoodSubsystem = new AdjustableHoodSubsystem();
  private final ServoTest m_servo = new ServoTest();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_indexer);

  XboxController m_driverController = new XboxController(OIconstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIconstants.kOperatorControllerPort);
  XboxController m_testController = new XboxController(OIconstants.kTestControllerPort);

  public static enum Mode{
    AUTO(false), COLLECTION(true), ALIGN(true), SHOOTING(true), DEFENSE(false);

    private boolean m_isShootCycle;

    private Mode(boolean isShootCycle){
      m_isShootCycle = isShootCycle;
    }

    public boolean isShootCycle(){
      //return COLLECTION || ALIGN || SHOOTING;
      return false;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_hoodSubsystem.setEncoderZeroPos(0);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_drive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        // Left Y Axis needs to be inverted for driving forward
        new RunCommand(() -> m_drive.arcadeDrive(-1 * m_operatorController.getRawAxis(OIconstants.leftYAxis),
            m_operatorController.getRawAxis(OIconstants.rightXAxis)), m_drive));

  }//comment

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * example new JoystickButton(m_operatorController,
     * Button.kBumperLeft.value).whenPressed(() ->
     * m_powerCell.enableIntakeAndIndexer()) .whenReleased(() ->
     * m_powerCell.disable());
     */

    /**
     * TEST CONTROLLER
     */
    // Indexer on left bumper
    new JoystickButton(m_testController, Button.kBumperLeft.value).whenPressed(() -> m_indexer.enable())
        .whenReleased(() -> m_indexer.disable());
    // Intake on right bumper
    //new JoystickButton(m_testController, Button.kBumperRight.value).whenPressed(() -> m_intake.enableIntake())
    //    .whenReleased(() -> m_intake.disableIntake());
    new JoystickButton(m_testController, Button.kBumperRight.value).whenPressed(() -> m_indexer.reverse())
        .whenReleased(() -> m_intake.disableIntake());
    // moving intake mover on right Y axis
    new RunCommand(() -> m_intake.rotateIntake(m_testController.getRawAxis(OIconstants.rightYAxis)));
    // moving loader on A button
    new JoystickButton(m_testController, Button.kA.value).whenPressed(() -> m_loader.enable())
        .whenReleased(() -> m_loader.disable());
    // moving shooter on A button
    // right side motor inverted (3)
    new JoystickButton(m_testController, Button.kB.value)
        .whenHeld(new RunShooter(m_leftShooterPID, m_rightShooterPID));
    
    // align camera on X button TODO
    //lets PID take over moving hood to test value on X button
    //new JoystickButton(m_testController, Button.kX.value).whenPressed(() -> new HoodPID(m_hoodSubsystem, 100));

    //sets the speed of hood to right Y axis;
    //new JoystickButton(m_testController, Axix).whenActive(() -> m_hoodSubsystem.setMotorVelo(0.3));    
    m_hoodSubsystem.setDefaultCommand(
      new RunCommand(() -> m_hoodSubsystem.setMotorVelo(m_testController.getRawAxis(OIconstants.leftYAxis)),m_hoodSubsystem));

    // new JoystickButton(m_driverController, XboxController.Button.kA.value).whenHeld(new RunCommand(() -> m_intake.moveIntakeUp(), m_intake));
    // new JoystickButton(m_driverController, XboxController.Button.kB.value).whenHeld(new RunCommand(() -> m_intake.moveIntakeDown(), m_intake));
    // Servo test on Y button
    new JoystickButton(m_testController, Button.kB.value).whenPressed(() -> m_servo.setBothAngle(ClimberConstants.degrees));
    
    /**
     * DRIVER CONTROLLER
     */
    // overall driving

    /**
     * OPERATOR CONTROLLER
     */
    // B button shooter
    new JoystickButton(m_operatorController, Button.kB.value)
        .whenHeld(new RunShooter(m_leftShooterPID, m_rightShooterPID));
    // left bumper indexer
    // Indexer on left bumper
    new JoystickButton(m_operatorController, Button.kBumperLeft.value).whenPressed(() -> m_indexer.enable())
        .whenReleased(() -> m_indexer.disable());
    // right bumper loader
    new JoystickButton(m_operatorController, Button.kBumperRight.value).whenPressed(() -> m_loader.enable())
        .whenReleased(() -> m_loader.disable());
    // Y button moves indexer and intake

    // left y-axis for hood
    // right y-axis for rotating intake or X button
    // A reverse everything

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    try {
      throw new Exception();
    } catch (Exception e) {
      System.out.println("No Auto Command");
      e.printStackTrace();
    }
    return null;
  }
}
