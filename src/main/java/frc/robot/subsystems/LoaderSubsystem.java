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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.LoaderConstants;
import frc.robot.Constants.SensorConstants;

public class LoaderSubsystem extends SubsystemBase {

  private CANSparkMax m_loader;
  private CANEncoder m_encoder;

  private DigitalInput m_shooterEmitter;
  private DigitalInput m_shooterReceiver;
  private DigitalInput m_bottomRightEmitter;
  private DigitalInput m_bottomRightReceiver;

  private DigitalInput m_middleEmitter;
  private DigitalInput m_middleReceiver;
  private DigitalInput m_topEmitter;
  private DigitalInput m_topReceiver;

  private int m_count;

  /**
   * Creates a new LoaderSubsystem.
   */
  public LoaderSubsystem() {
    m_count = 0;

    m_loader = new CANSparkMax(LoaderConstants.kLoaderMotor, MotorType.kBrushless);
    m_loader.setInverted(true);

    m_encoder = new CANEncoder(m_loader);
    m_encoder.setPosition(0);
    m_encoder.setVelocityConversionFactor(0.2083);

    m_topEmitter = new DigitalInput(SensorConstants.topEmitter);
    m_topReceiver = new DigitalInput(SensorConstants.topReciever);

    m_middleEmitter = new DigitalInput(SensorConstants.middleEmitter);
    m_middleReceiver = new DigitalInput(SensorConstants.middleReciever);

    m_shooterEmitter = new DigitalInput(SensorConstants.ShooterEmitter);
    m_shooterReceiver = new DigitalInput(SensorConstants.ShooterReciever);
    m_bottomRightEmitter = new DigitalInput(SensorConstants.bottomEmitter);
    m_bottomRightReceiver = new DigitalInput(SensorConstants.bottomReciever);

  }

  public void enable() {
    m_loader.set(LoaderConstants.kFeedLoaderSpeed);
  }

  public void reverse(double speed) {
    m_loader.set(-speed);
  }

  public void enable(double speed) {
    m_loader.set(speed);
  }

  public void disable() {
    m_loader.set(0);
  }

  public void periodic() {

    // counting
    if (isBottomBroken()) {
      m_count++;
    }
    if (isShooterBeamBroken()) {
      m_count--;
    }

    if (DashboardConstants.kLoaderTelemetry) {
      SmartDashboard.putBoolean("Top Emitter", m_topEmitter.get());
      SmartDashboard.putBoolean("Top Reciever", m_topReceiver.get());
      SmartDashboard.putBoolean("Middle Emitter", m_middleEmitter.get());
      SmartDashboard.putBoolean("Middle Reciever", m_middleReceiver.get());

      SmartDashboard.putBoolean("Bottom Emitter", m_bottomRightEmitter.get());
      SmartDashboard.putBoolean("Bottom Reciever", m_bottomRightReceiver.get());
      SmartDashboard.putBoolean("Shooter Emitter", m_shooterEmitter.get());
      SmartDashboard.putBoolean("Shooter Reciever", m_shooterReceiver.get());

      SmartDashboard.putNumber("# of balls", getCount());

      SmartDashboard.putNumber("Loader Out", m_loader.get());
      SmartDashboard.putNumber("Loader RPM", m_encoder.getVelocity());
    }
  }

  public int getCount() {

    return m_count;
  }

  public boolean isShooterBeamBroken() {
    return !(m_shooterReceiver.get());
    // return false;
  }

  public boolean isTopBeamBroken() {
    return !(m_topReceiver.get());
    // return false;
  }

  public boolean isMiddleBeamBroken() {
    return !(m_middleReceiver.get());
    // return false;
  }

  public boolean isBottomBroken() {
    return !(m_bottomRightReceiver.get());
  }

}
