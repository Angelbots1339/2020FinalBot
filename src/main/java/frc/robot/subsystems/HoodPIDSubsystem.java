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

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.HoodConstants;

public class HoodPIDSubsystem extends PIDSubsystem {
  private final CANSparkMax m_hood;
  private final CANEncoder m_encoder;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(HoodConstants.KSVolts,
      HoodConstants.KVVoltSecondsPerRotation);

  /**
   * Creates a new HoodPID.
   */
  public HoodPIDSubsystem(double setpoint) {
    super(new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD));
    m_hood = new CANSparkMax(HoodConstants.kHoodPort, MotorType.kBrushless);
    m_encoder = new CANEncoder(m_hood);
    m_encoder.setPosition(0);

    getController().setTolerance(HoodConstants.kPositionTolerance);

    // Regardless of what's passed in, clamp to the min and max
    MathUtil.clamp(setpoint, HoodConstants.kMinEncoderValue, HoodConstants.kmaxEncoderValue);
    setSetpoint(setpoint);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    output += m_feedforward.calculate(setpoint);
    output = MathUtil.clamp(output, -HoodConstants.kMaxHoodVolt, HoodConstants.kMaxHoodVolt);
    m_hood.setVoltage(output);
    SmartDashboard.putNumber("HoodOutput", output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_encoder.getPosition();
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  public void setSetpoint(double setpoint) {
    // Regardless of what's passed in, clamp to the min and max
    setpoint = MathUtil.clamp(setpoint, HoodConstants.kMinEncoderValue,
        HoodConstants.kmaxEncoderValue);
    super.setSetpoint(setpoint);
  }

  public void periodic() {
    super.periodic();
    if (m_hood.getOutputCurrent() > HoodConstants.kMinResistedVoltage) {
      m_encoder.setPosition(m_hood.getAppliedOutput() > 0 ? HoodConstants.kmaxEncoderValue
          : HoodConstants.kMinEncoderValue);
    }
    if (DashboardConstants.kHoodPIDTelemetry) {
      SmartDashboard.putNumber("HoodEncoder", getMeasurement());
      SmartDashboard.putNumber("HoodSet", getController().getSetpoint());
      SmartDashboard.putNumber("Hood Current", m_hood.getOutputCurrent());
      SmartDashboard.putNumber("Hood Applied Output", m_hood.getAppliedOutput());
      SmartDashboard.putNumber("Hood Motor Temp", m_hood.getMotorTemperature());
    }
  }
}
