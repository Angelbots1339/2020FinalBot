package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.utils.DriveControl;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private WPI_TalonFX m_leftFront = new WPI_TalonFX(DriveConstants.kLeftFrontMotor);
  private WPI_TalonFX m_leftBack = new WPI_TalonFX(DriveConstants.kLeftBackMotor);

  // The motors on the right side of the drive.
  private WPI_TalonFX m_rightFront = new WPI_TalonFX(DriveConstants.kRightFrontMotor);
  private WPI_TalonFX m_rightBack = new WPI_TalonFX(DriveConstants.kRightBackMotor);

  // Group them
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftFront, m_leftBack);
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightFront, m_rightBack);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {

    // Practice bot is wired and mounted in a way where nothing is inverted
    // Left Axis of joystick response needs to be inverted though

    // Wooooah there - Chad implemented this for speed control on initial testing
    setMaxOutput(DriveConstants.kMaxDriveSpeed);
    m_drive.setDeadband(DriveConstants.kMinPower);

    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_leftFront.configOpenloopRamp(0);
    m_leftBack.configOpenloopRamp(0);
    m_rightFront.configOpenloopRamp(0);
    m_rightBack.configOpenloopRamp(0);
  }

  /**
   * Drives the robot using curvature controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void curvatureDrive(double fwd, double rot) {
    // speed limiting accomplished in Constructor
    m_drive.curvatureDrive(fwd, rot, Math.abs(fwd) < DriveConstants.kTurnInPlaceThreshold);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    // speed limiting accomplished in Constructor
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * creates an arcade drive with a threshold for aliging
   * 
   * @param fwd
   * @param rot
   * @param threshold
   */
  public boolean arcadeDrive(double fwd, double rot, double moveThreshold, double reportThreshold) {
    return tankDrive(fwd + rot, fwd - rot, moveThreshold, reportThreshold);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public boolean tankDrive(double left, double right, double moveThreshold, double reportThreshold) {
    double leftSpeed = Math.abs(left) > moveThreshold
        ? Math.copySign(Math.max(DriveConstants.kMinPower, Math.abs(left)), left)
        : 0;
    double rightSpeed = Math.abs(right) > moveThreshold
        ? Math.copySign(Math.max(DriveConstants.kMinPower, Math.abs(right)), right)
        : 0;

    m_drive.tankDrive(leftSpeed, rightSpeed, false);
    return Math.abs(left) < reportThreshold && Math.abs(right) < reportThreshold;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getLeftMeters() {
    return DriveConstants.kMetersPerClick
        * (m_leftFront.getSelectedSensorPosition() + m_leftBack.getSelectedSensorPosition()) / 2;
  }

  public double getRightMeters() {
    return DriveConstants.kMetersPerClick
        * (m_rightFront.getSelectedSensorPosition() + m_rightBack.getSelectedSensorPosition()) / 2;
  }

  public void periodic() {
    if (DashboardConstants.kDriveTelemetry) {
      SmartDashboard.putNumber("left front speed", m_leftFront.get());
      SmartDashboard.putNumber("right front speed", m_rightFront.get());

      SmartDashboard.putNumber("left front temp", m_leftFront.getTemperature());
      SmartDashboard.putNumber("right front temp", m_rightFront.getTemperature());

      SmartDashboard.putNumber("left front volt", m_leftFront.getBusVoltage());
      SmartDashboard.putNumber("right front volt", m_rightFront.getBusVoltage());

      SmartDashboard.putNumber("left front percent output", m_leftFront.getMotorOutputPercent());
      SmartDashboard.putNumber("right front percent output", m_rightFront.getMotorOutputPercent());

      SmartDashboard.putNumber("left position", getLeftMeters());
      SmartDashboard.putNumber("right position", getRightMeters());
    }

    if (DashboardConstants.kExcessDriveTelemetry) {
      SmartDashboard.putNumber("left back speed", m_rightBack.get());
      SmartDashboard.putNumber("right back speed", m_rightFront.get());

      SmartDashboard.putNumber("left back temp", m_rightBack.getTemperature());
      SmartDashboard.putNumber("right back temp", m_rightFront.getTemperature());

      SmartDashboard.putNumber("left back volt", m_rightBack.getBusVoltage());
      SmartDashboard.putNumber("right back volt", m_rightFront.getBusVoltage());

      SmartDashboard.putNumber("left back percent output", m_rightBack.getMotorOutputPercent());
      SmartDashboard.putNumber("right back percent output", m_rightFront.getMotorOutputPercent());
    }
  }

  public void curvatureDrive(DriveControl driveControl) {
    curvatureDrive(driveControl.getDrive(), driveControl.getTurn());
  }
}
