package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DriveConstants;

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

  /*
   * // The left-side drive encoder private final Encoder m_leftEncoder = new
   * Encoder(DriveConstants.kLeftEncoderPorts[0],
   * DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);
   * 
   * // The right-side drive encoder private final Encoder m_rightEncoder = new
   * Encoder(DriveConstants.kRightEncoderPorts[0],
   * DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);
   * 
   * // The gyro sensor //private final Gyro m_gyro = new ADXRS450_Gyro();
   */

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // m_leftBack.follow(m_leftFront);
    // m_rightBack.follow(m_rightBack);

    // Practice bot is wired and mounted in a way where nothing is inverted
    // Left Axis of joystick response needs to be inverted though

    // Wooooah there - Chad implemented this for speed control on initial testing
    setMaxOutput(DriveConstants.kMaxDriveSpeed);
    m_drive.setDeadband(DriveConstants.kMinPower);
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
  public boolean arcadeDrive(double fwd, double rot, double threshold) {
    return tankDrive(fwd + rot, fwd - rot, threshold);
  }

  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  public boolean tankDrive(double left, double right, double threshold) {
    SmartDashboard.putNumber("left Vision", left);
    SmartDashboard.putNumber("right Vision", right);
    double leftSpeed = Math.abs(left) > threshold
        ? Math.copySign(Math.max(DriveConstants.kMinPower, Math.abs(left)), left)
        : 0;
    double rightSpeed = Math.abs(right) > threshold
        ? Math.copySign(Math.max(DriveConstants.kMinPower, Math.abs(right)), right)
        : 0;
    SmartDashboard.putNumber("real left Vision", leftSpeed);
    SmartDashboard.putNumber("real right Vision", rightSpeed);

    m_drive.tankDrive(leftSpeed, rightSpeed, false);
    // m_leftMotors.set(leftSpeed);
    // m_rightMotors.set(rightSpeed);
    return Math.abs(left) < threshold && Math.abs(right) < threshold;
  }
  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  /*
   * public void resetEncoders() { //m_leftEncoder.reset();
   * //m_rightEncoder.reset(); }
   */
  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  /*
   * public double getAverageEncoderDistance() { return
   * (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0; }
   */

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  /*
   * public Encoder getLeftEncoder() { return m_leftEncoder; }
   */
  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  /*
   * public Encoder getRightEncoder() { return m_rightEncoder; }
   */

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
   * Zeroes the heading of the robot.
   */
  /*
   * public void zeroHeading() { m_gyro.reset(); }
   */

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  /*
   * public double getHeading() { return Math.IEEEremainder(m_gyro.getAngle(),
   * 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0); }
   */
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  /*
   * public double getTurnRate() { return m_gyro.getRate() *
   * (DriveConstants.kGyroReversed ? -1.0 : 1.0); }
   */

  public void periodic() {
    if (DashboardConstants.kDriveTelemetry) {
      SmartDashboard.putNumber("left front speed", m_leftFront.get());
      SmartDashboard.putNumber("left back speed", m_rightBack.get());
      SmartDashboard.putNumber("right front speed", m_rightFront.get());
      SmartDashboard.putNumber("right back speed", m_rightFront.get());

      SmartDashboard.putNumber("left front temp", m_leftFront.getTemperature());
      SmartDashboard.putNumber("left back temp", m_rightBack.getTemperature());
      SmartDashboard.putNumber("right front temp", m_rightFront.getTemperature());
      SmartDashboard.putNumber("right back temp", m_rightFront.getTemperature());

      SmartDashboard.putNumber("left front volt", m_leftFront.getBusVoltage());
      SmartDashboard.putNumber("left back volt", m_rightBack.getBusVoltage());
      SmartDashboard.putNumber("right front volt", m_rightFront.getBusVoltage());
      SmartDashboard.putNumber("right back volt", m_rightFront.getBusVoltage());

      SmartDashboard.putNumber("left front percent output", m_leftFront.getMotorOutputPercent());
      SmartDashboard.putNumber("left back percent output", m_rightBack.getMotorOutputPercent());
      SmartDashboard.putNumber("right front percent output", m_rightFront.getMotorOutputPercent());
      SmartDashboard.putNumber("right back percent output", m_rightFront.getMotorOutputPercent());
    }

  }
}
