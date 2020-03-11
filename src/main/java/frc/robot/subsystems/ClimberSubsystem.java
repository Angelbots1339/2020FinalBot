/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DashboardConstants;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

  private CANSparkMax m_rightClimber;
  private CANSparkMax m_leftClimber;
  private double m_speed;
  private boolean m_enabled = false;

  public ClimberSubsystem() {
    m_rightClimber = new CANSparkMax(ClimberConstants.kRightClimberMotor, MotorType.kBrushless);
    m_leftClimber = new CANSparkMax(ClimberConstants.kLeftClimberMotor, MotorType.kBrushless);

    m_leftClimber.setInverted(true);
    m_rightClimber.setInverted(false);

    m_leftClimber.setIdleMode(IdleMode.kBrake);
    m_rightClimber.setIdleMode(IdleMode.kBrake);
  }

  public boolean isEnabled() {
    return m_enabled;
  }

  public void enable() {
    m_enabled = true;
  }

  public void setSpeed(double speed) {
    m_rightClimber.set(speed);
    m_leftClimber.set(speed);
  }

  public void run() {
    if (m_enabled)
      setSpeed(-ClimberConstants.kClimberSpeed);
  }

  @Override
  public void periodic() {
    if (DashboardConstants.kClimberTelemetry) {
      SmartDashboard.putNumber("Left Climb Amp", m_leftClimber.getOutputCurrent());
      SmartDashboard.putNumber("Right Climb Amp", m_rightClimber.getOutputCurrent());
      SmartDashboard.putNumber("Climber Speed", m_speed);
    }
    // This method will be called once per scheduler run
  }

  public void stop() {
    setSpeed(0);
  }

  public void runInverted() {
    if (m_enabled)
      setSpeed(ClimberConstants.kClimberResetSpeed);
  }
}
