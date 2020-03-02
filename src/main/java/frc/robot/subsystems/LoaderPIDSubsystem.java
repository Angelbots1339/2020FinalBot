/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.SensorConstants;

public class LoaderPIDSubsystem extends PIDSubsystem {

  private CANSparkMax m_loader;
  private final CANEncoder m_encoder;

  private DigitalInput m_shooterEmitter;
  private DigitalInput m_shooterReceiver;
  private DigitalInput m_bottomRightEmitter;
  private DigitalInput m_bottomRightReceiver;

  private DigitalInput m_middleEmitter;
  private DigitalInput m_middleReceiver;
  private DigitalInput m_topEmitter;
  private DigitalInput m_topReceiver;

  private int m_count;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(LoaderConstants.KSVolts,
    LoaderConstants.KVVoltSecondsPerRotation);

  /**
   * Creates a new LoaderSubsystem.
   */
  public LoaderPIDSubsystem() {
    super(new PIDController(LoaderConstants.kP, LoaderConstants.kI, LoaderConstants.kD));
    m_count = 0;

    m_loader = new CANSparkMax(LoaderConstants.kLoaderMotor, MotorType.kBrushless);
    m_loader.setInverted(true);

    m_encoder = new CANEncoder(m_loader);
    m_encoder.setPosition(0);
    m_encoder.setVelocityConversionFactor(0.2083);

    getController().setTolerance(LoaderConstants.kLoaderToleranceRPS);
    setSetpoint(LoaderConstants.kLoaderSetpoint);

    m_topEmitter = new DigitalInput(SensorConstants.topEmitter);
    m_topReceiver = new DigitalInput(SensorConstants.topReciever);

    m_middleEmitter = new DigitalInput(SensorConstants.middleEmitter);
    m_middleReceiver = new DigitalInput(SensorConstants.middleReciever);

    m_shooterEmitter = new DigitalInput(SensorConstants.ShooterEmitter);
    m_shooterReceiver = new DigitalInput(SensorConstants.ShooterReciever);
    m_bottomRightEmitter = new DigitalInput(SensorConstants.bottomEmitter);
    m_bottomRightReceiver = new DigitalInput(SensorConstants.bottomReciever);
  }


  public void reverse(double speed) {
    m_loader.set(-speed);
  }

  public void runSpeed(double speed) {
    m_loader.set(speed);
  }

  public int getCount() {
    return m_count;
  }

  public boolean isShooterBeamBroken() {
    return !(m_shooterReceiver.get());
  }

  public boolean isTopBeamBroken() {
    return !(m_topReceiver.get());
  }

  public boolean isMiddleBeamBroken() {
    return !(m_middleReceiver.get());
  }

  public boolean isBottomBroken() {
    return false; // TODO needs to be corrected
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    output += m_feedforward.calculate(setpoint);
    //output = MathUtil.clamp(output, -6, 6);
    m_loader.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return m_encoder.getVelocity();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    super.setSetpoint(setpoint);
  }

  
  public void periodic() {
    super.periodic();

    // counting
    if (isBottomBroken()) {
      m_count++;
    }
    if (isShooterBeamBroken()) {
      m_count--;
    }
    if (DashboardConstants.kPIDLoaderTelemetry) {
      SmartDashboard.putBoolean("Top Emitter", m_topEmitter.get());
      SmartDashboard.putBoolean("Top Reciever", m_topReceiver.get());
      SmartDashboard.putBoolean("Middle Emitter", m_middleEmitter.get());
      SmartDashboard.putBoolean("Middle Reciever", m_middleReceiver.get());

      SmartDashboard.putBoolean("Bottom Emitter", m_bottomRightEmitter.get());
      SmartDashboard.putBoolean("Bottom Reciever", m_bottomRightReceiver.get());
      SmartDashboard.putBoolean("Shooter Emitter", m_shooterEmitter.get());
      SmartDashboard.putBoolean("Shooter Reciever", m_shooterReceiver.get());

      SmartDashboard.putNumber("# of balls", getCount());

      // This method will be called once per scheduler run
      SmartDashboard.putNumber("Loader Out", m_loader.get());
      SmartDashboard.putNumber("Loader RPM", m_encoder.getVelocity());
    }
  }

}
