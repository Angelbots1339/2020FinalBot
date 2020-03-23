package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorConstants;
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
  // The robot's Gyro for angles
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry m_odometry;

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

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Returns the heading of the robot
   * 
   * @return heading in degress, -180 to 180
   */
  private double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), AngleConstants.kFullTurn) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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

  /**
   * Tank Drive based on a voltage, required for trajectory code
   * 
   * @param leftVolts
   * @param rightVolts
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts); // TODO - Double check need for negative volts
    m_drive.feed();
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

  public void stop() {
    tankDrive(0, 0);
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

  /**
   * Get Left Drive-side Encoder Values
   * 
   * @return Average Left Encoder values in meters
   */
  public double getLeftMeters() {
    return DriveConstants.kMetersPerClick
        * Utils.average(m_leftFront.getSelectedSensorPosition(), m_leftBack.getSelectedSensorPosition());
  }

  /**
   * Get Right Drive-side Encoder Values, sign is inverted based on drivetrain
   * setup
   * 
   * @return Average Right Encoder Values in meters
   */
  public double getRightMeters() {
    return DriveConstants.kMetersPerClick * -1
        * Utils.average(m_rightFront.getSelectedSensorPosition(), m_rightBack.getSelectedSensorPosition());
  }

  public double getForwardMeters() {
    return Utils.average(getLeftMeters(), getRightMeters());
  }

  /**
   * TODO - CHECK this calc
   * 
   * @return
   */
  public double getLeftVelocityMeters() {
    return DriveConstants.kMetersPerClick * m_leftFront.getSelectedSensorVelocity() / MotorConstants.kFalconUpdateTime;
  }

  public double getRightVelocityMeters() {
    return DriveConstants.kMetersPerClick * -1 * m_rightFront.getSelectedSensorVelocity() / MotorConstants.kFalconUpdateTime;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getRightVelocityMeters(), getLeftVelocityMeters());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * 
   * @return gyro angle (deg)
   */
  public double getRotation() {
    return m_gyro.getAngle();
  }

  /**
   * Zeroes the heading (gyro) of the robot
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void resetEncoders() {
    m_rightBack.setSelectedSensorPosition(0);
    m_rightFront.setSelectedSensorPosition(0);
    m_leftBack.setSelectedSensorPosition(0);
    m_leftFront.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftMeters(), getRightMeters());

    if (DashboardConstants.kDriveTelemetry) {
      SmartDashboard.putNumber("left front set speed", m_leftFront.get());
      SmartDashboard.putNumber("right front set speed", m_rightFront.get());

      SmartDashboard.putNumber("left front temp", m_leftFront.getTemperature());
      SmartDashboard.putNumber("right front temp", m_rightFront.getTemperature());

      SmartDashboard.putNumber("left front volt", m_leftFront.getBusVoltage());
      SmartDashboard.putNumber("right front volt", m_rightFront.getBusVoltage());

      SmartDashboard.putNumber("left front percent output", m_leftFront.getMotorOutputPercent());
      SmartDashboard.putNumber("right front percent output", m_rightFront.getMotorOutputPercent());

      SmartDashboard.putNumber("left position", getLeftMeters());
      SmartDashboard.putNumber("right position", getRightMeters());
      SmartDashboard.putNumber("left vel", getLeftVelocityMeters());
      SmartDashboard.putNumber("right vel", getRightVelocityMeters());
      SmartDashboard.putNumber("gyro rotation", getRotation());

    }

    if (DashboardConstants.kExcessDriveTelemetry) {
      SmartDashboard.putNumber("left back set speed", m_rightBack.get());
      SmartDashboard.putNumber("right back set speed", m_rightFront.get());

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
